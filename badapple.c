const char badapple[] __attribute__((section(".text"))) = {
    #include "./badapple_1bit.hex"
};
const int badapple_size = sizeof(badapple) / (22*3);
