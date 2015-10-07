# SWAVR
#### A software Implementation of an Atmel&trade; AVR&trade; written in C

There is a `.h` file and accompanying `.c` file for each supported microcontroller. Include the header file(s) corresponding to the microcontroller(s) you wish to implement in your project.

#### Simple Useage

In the simplest use case there are only a few functions to worry about, taking the ATmega128 as an example:

```
void atmega128_init(struct atmega128 * const mega,
                 void(* const uart0_write_cb) (void*, uint8_t),
                 void* const uart0_write_cb_arg);
```

Pass a pointer to an uninitialised `struct atmega128` into this function for initialisation. `uart0_write_cb` is called from `atmega128_tick` when the AVR would have outputted a UART character, `uart0_write_cb_arg` will become the `void*` argument to the aforementioned callback.


```
void atmega128_tick(struct atmega128 * const mega);
```

Executes and instruction


```
void atmega128_uart0_write(struct atmega128 * const mega,
                            const uint8_t value);
```

Writes `value` to the uart0 port of the MCU


```
int atmega128_load_hex(struct atmega128 * const mega,
                       const char* const filename);
```

Load an Intel hex file into the flash memory of the AVR

##### Example
```
void uart0_callback(void* arg, uint8_t c)
{
    unsigned char* done = (unsigned char*) arg;
    putchar(c);
    if (c == '\n')
    {
        *done = 1;
    }
}

int print_one_line(const char* const hex_path)
{
    unsigned char done = 0;

    // allocate memory
    struct atmega128 mega;

    // initialise memory and registers
    atmega128_init(&mega, &uart0_callback, &done);

    // load hex file from the path passed into this function as a parameter
    if (atmega128_load_hex(&mega, hex_path))
    {
        fprintf(stderr, "failed to open hex file\n");
        return -1;
    }

    while (done == 0)
    {
        // the tick function will call uart0_callback with any UART output
        // this will continue until the character outputted is a newline
        // at which point done will be set to 1
        atmega128_tick(&mega);
    }
    return 0;
}
```

#### Implementing an Unsupported AVR or Extending the Functionality of an Existing One

The core functionality of the AVR simulator is implemented in `avr.c`. Extensions to the core functionality are done through manipulation of the IO registers.

There is an array of callbacks used to notify any peripheral modules of changes to the IO modules (the array is `avr.dmem.callbacks` where `avr` is a `struct avr`). This array contains 2 callbacks for each IO register, one of these is called whenever the corresponding IO register is read, and the other is called when it is written.