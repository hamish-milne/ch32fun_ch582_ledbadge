#pragma once

#if __STDC_VERSION__ < 201112L
#error "This project requires at least the C11 standard version."
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <limits.h>

#ifdef __GNUC__
#define PURE __attribute__((pure))
#define CONST __attribute__((const))
#define NONNULL __attribute__((nonnull))
#define RETURNS_NONNULL __attribute__((returns_nonnull))
#else
#define PURE
#define CONST
#define NONNULL
#define RETURNS_NONNULL
#endif

// Priority levels for scheduled tasks
typedef enum {
    PRIORITY_REAL = 0,
    PRIORITY_HIGH,
    PRIORITY_NORMAL,
    PRIORITY_LOW,
    PRIORITY_COUNT,
} priority_t;

typedef uint8_t priority_mask_t;
static_assert(PRIORITY_COUNT <= CHAR_BIT*sizeof(priority_mask_t), "PRIORITY_COUNT exceeds size of priority_mask_t");
#define PRIORITY_MASK_MAX (1 << (PRIORITY_COUNT - 1))

CONST static inline priority_mask_t PRIORITY_MASK(priority_t priority) {
    return (PRIORITY_MASK_MAX >> priority);
}

// System clock type. Must be unsigned to avoid UB on wrap-around.
typedef uint32_t sysclk_t;
static_assert((sysclk_t)(-1) > 0, "sysclk_t must be an unsigned type");

// Index type for ring buffer. Must be unsigned to avoid UB on wrap-around.
typedef uint32_t index_t;
static_assert((index_t)(-1) > 0, "index_t must be an unsigned type");

// Ring buffer capacity; the maximum number of tasks per priority queue.
#define RB_CAPACITY 0x40
static_assert((RB_CAPACITY & (RB_CAPACITY - 1)) == 0, "RB_CAPACITY must be a power of 2");
#define RB_MASK (RB_CAPACITY - 1)

/*
If the difference between the current system clock and the queue watermark exceeds CLOCK_LAG_MAX,
the queue has been starved of CPU time for too long, which we consider a non-recoverable error.
If we let it keep going, eventually the clock will wrap around and tasks will start executing
out of order.
*/
#define CLOCK_LAG_MAX 0x10000000

// Minimum delay for scheduled tasks, in sysclk_t units
#define TASK_DELAY_MIN 1000
static_assert(TASK_DELAY_MIN < CLOCK_LAG_MAX, "TASK_DELAY_MIN must be less than CLOCK_LAG_MAX");

// Task handler function type
typedef void (*handler_t)(void *ctx, sysclk_t scheduled_time);

// Task structure
typedef struct {
    handler_t   handler;
    void       *context;
    sysclk_t    scheduled_time;
} task_t;

// Ring buffer structure
typedef struct {
    task_t   buffer[RB_CAPACITY];
    index_t  head;
    index_t  tail;
    sysclk_t watermark;
} ringbuffer_t;

// Scheduler structure
typedef struct {
    ringbuffer_t    queues[PRIORITY_COUNT];
    priority_mask_t mask;
    sysclk_t        now;
} scheduler_t;

typedef enum {
    SCHEDULER_OK = 0,
    SCHEDULER_ERR_EMPTY,
    SCHEDULER_ERR_FULL,
    SCHEDULER_ERR_CLOCK_LAG,
} scheduler_result_t;

typedef enum {
    IRQ_DISABLED,
    IRQ_ENABLED,
} irq_state_t;

// Enable or disable interrupts, returning the previous interrupt state
irq_state_t scheduler_irq_set(irq_state_t state);

// Schedule the next interrupt at the given system clock time
void scheduler_irq_schedule(sysclk_t alarm);

sysclk_t scheduler_now(void);

NONNULL static void ringbuffer_init(ringbuffer_t *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->watermark = 0;
}

NONNULL void scheduler_init(scheduler_t *scheduler) {
    for (priority_t i = PRIORITY_REAL; i < PRIORITY_COUNT; i++) {
        ringbuffer_init(&scheduler->queues[i]);
    }
    scheduler->mask = 0;
    scheduler->now = 0;
}

PURE NONNULL RETURNS_NONNULL static inline task_t * ringbuffer_at(ringbuffer_t *queue, index_t i) {
    return &queue->buffer[i & RB_MASK];
}

CONST static inline bool time_before(sysclk_t watermark, sysclk_t a, sysclk_t b) {
    // We return < here to force an unsigned comparison, handling wrap-around correctly
    return (a - watermark) < (b - watermark);
}

NONNULL static scheduler_result_t scheduler_enqueue(scheduler_t *scheduler, priority_t priority, const task_t *task) {
ringbuffer_t *queue = &scheduler->queues[priority];

    if ((queue->head - queue->tail) == RB_CAPACITY) {
        return SCHEDULER_ERR_FULL;
    }

    index_t insert_pos = queue->head;
    // The use of != is deliberate to handle wrap-around correctly
    while (insert_pos != queue->tail && time_before(queue->watermark, task->scheduled_time, ringbuffer_at(queue, insert_pos - 1)->scheduled_time)) {
        insert_pos--;
    }

    // Decide which direction to shift elements, to minimize the number of moves
    if ((queue->head - insert_pos) < (insert_pos - queue->tail)) {
        for (index_t i = queue->tail; i != insert_pos; i++) {
            *ringbuffer_at(queue, i - 1) = *ringbuffer_at(queue, i);
        }
        // Insert at tail
        queue->tail--;
    } else {
        for (index_t i = queue->head; i != insert_pos; i--) {
            *ringbuffer_at(queue, i) = *ringbuffer_at(queue, i - 1);
        }
        // Insert at head
        queue->head++;
    }
    *ringbuffer_at(queue, insert_pos) = *task;
    return SCHEDULER_OK;
}

NONNULL static scheduler_result_t ringbuffer_peek(ringbuffer_t *queue, task_t *task) {
    if (queue->head == queue->tail) {
        // Buffer is empty
        return SCHEDULER_ERR_EMPTY;
    }
    *task = *ringbuffer_at(queue, queue->tail);
    return SCHEDULER_OK;
}

NONNULL static scheduler_result_t scheduler_dequeue(scheduler_t *scheduler, task_t *out_task, priority_t *out_priority, sysclk_t *out_alarm) {
    *out_alarm = scheduler->now - 1;
    // For every priority level higher than the currenly executing one...
    for (priority_t i = PRIORITY_REAL; (i < PRIORITY_COUNT) && (scheduler->mask < PRIORITY_MASK(i)); i++) {
        ringbuffer_t *queue = &scheduler->queues[i];
        if (ringbuffer_peek(queue, out_task) == SCHEDULER_OK) {
            // If the task is ready to run...
            if (time_before(queue->watermark, out_task->scheduled_time, scheduler->now + TASK_DELAY_MIN)) {
                // printf("%u, %u, %u, %u, %u\n", queue->watermark, scheduler->now, out_task->scheduled_time, scheduler->now - queue->watermark, out_task->scheduled_time - queue->watermark);
                // Dequeue the task
                queue->tail++;
                queue->watermark = out_task->scheduled_time + 1;
                *out_priority = i;
                // Stop here, because a lower priority task cannot preempt a higher one
                return SCHEDULER_OK;
                // Otherwise, if it's due sooner than the current candidate...
            } else if (time_before(scheduler->now, out_task->scheduled_time, *out_alarm)) {
                // printf("Next alarm candidate at priority %d scheduled for %u\n", i, (unsigned int)out_task->scheduled_time);
                // Update the next alarm time
                *out_alarm = out_task->scheduled_time;
            }
        }
    }
    out_task->handler = NULL;
    return SCHEDULER_ERR_EMPTY;
}

// Run the highest priority ready task once. This assumes interrupts are disabled.
NONNULL static scheduler_result_t scheduler_tick(scheduler_t *scheduler) {
    task_t next_task;
    priority_t next_priority;
    sysclk_t next_alarm;
    scheduler_result_t result = scheduler_dequeue(scheduler, &next_task, &next_priority, &next_alarm);
    if (result > SCHEDULER_ERR_EMPTY) {
        return result;
    }
    if (next_alarm != scheduler->now - 1) {
        // If next_alarm was set, this means there is a higher priority task, but it's not ready yet.
        scheduler_irq_schedule(next_alarm);
    }
    if (next_task.handler != NULL) {
        // Mark the task's priority as executing
        scheduler->mask |= PRIORITY_MASK(next_priority);
        irq_state_t prev_state = scheduler_irq_set(IRQ_ENABLED);
        next_task.handler(next_task.context, next_task.scheduled_time);
        scheduler_irq_set(prev_state);
        scheduler->mask &= ~PRIORITY_MASK(next_priority);
    }
    return result;
}

// Run all ready tasks. This assumes interrupts are disabled.
NONNULL static scheduler_result_t scheduler_tick_all(scheduler_t *scheduler) {
    scheduler_result_t result = SCHEDULER_OK;
    while (result == SCHEDULER_OK) {
        // printf("%u\n", (unsigned int)clock_time);
        scheduler->now = scheduler_now();
        result = scheduler_tick(scheduler);
    }
    return result > SCHEDULER_ERR_EMPTY ? result : SCHEDULER_OK;
}

NONNULL static scheduler_result_t scheduler_task(scheduler_t *scheduler, priority_t priority, const task_t *task) {
    // Disable interrupts while modifying the scheduler
    irq_state_t prev_state = scheduler_irq_set(IRQ_DISABLED);
    scheduler_result_t result = scheduler_enqueue(scheduler, priority, task);
    if (result == SCHEDULER_OK && scheduler->mask < PRIORITY_MASK(priority)) {
        // If the newly scheduled task has higher priority than the currently executing one, tick the scheduler
        result = scheduler_tick_all(scheduler);
    }
    scheduler_irq_set(prev_state);
    return result;
}
