const char font[] __attribute__((section(".text"))) = {
    #include "./font.hex"
};
const int font_size = sizeof(font);
