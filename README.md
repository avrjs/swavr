# SWAVR
#### A software Implementation of an Atmel&trade; AVR&trade; written in C

There is a `.h` file and accompanying `.c` file for each supported microcontroller. Include the header file(s) corresponding to the microcontroller(s) you wish to implement in your project.

#### Simple Useage

In the simplest use case there are only a few functions to worry about, taking the ATmega128 as an example:

```
struct atmega128_callbacks
{
    void (*sleep)(void*, uint8_t);
    void* sleep_arg;
    void(*uart0)(void*, uint8_t);
    void* uart0_arg;
};
```

`uart0` is called when the AVR would have outputted a UART character, `uart0_arg` will become the first argument and the second argument is the character transmitted.
`sleep` is called when the microcontroller enters or wakes from sleep. `sleep_arg` will become the first argument and the second argument to `sleep` is `1` when entering sleep and `0` when waking.

```
struct atmega128_config
{
    unsigned char bootsz;
    unsigned char bootrst;
};
```

`bootsz` indicates the size of the boot flash section, `bootrst` changes the reset vector, information on both these can be found in the ATmega128 datasheet.

```
void atmega128_init(struct atmega128 * const mega, const struct atmega128_callbacks callbacks, const struct atmega128_config config);
```

Pass a pointer to an uninitialised `struct atmega128` into this function for initialisation.

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

    struct atmega128_callbacks callbacks = {
        .uart0 = &uart0_callback,
        .uart0_arg = &done,
        .sleep = 0, // will not sleep
        .sleep_arg = 0
    };

    const struct atmega128_config config = {;
        .bootsz = 3,
        .bootrst = 0
    };

    // initialise memory and registers
    atmega128_init(&mega, callbacks, config);

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
