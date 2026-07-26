/* Keep Keil flash-loader writes aligned to the MSPM0 ECC programming width. */
__attribute__((section(".flash_padding"), used, aligned(8)))
unsigned char flash_download_padding[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
