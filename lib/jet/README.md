
# jet: Jon's Embedded Toolkit

A library of simple data structures tuned for embedded use.

## Welcome!

This library provides some simple data structures that fit my preferences for embedded development. The principles include:

* Simple algorithms (small code size, easy maintenance)
* Small memory usage
* Static memory usage options (hard upper bounds, easy static analysis)
* Limited polymorphism and inheritance (vtables SUCK for low-power, low-mem devices)

## Usage

```C++
#include <jet.h>

// PointerList

PointerList<MyStruct> list(4); // list of pointers to instances of MyStruct with initial size of 4

MyStruct* buf[15];
PointerList<MyStruct> list(buf, 15); // list of pointers to instances of MyStruct using external array
```

## Documentation

After this file, the source is probably your best bet.

## Contributing

This library lives on [GitHub](https://github.com/jkunkee/AirQualSniff/tree/main/lib/jet) and [Wokwi](https://wokwi.com/projects/364666417566269441).

Remember, the Particle CLI has the command `particle compile examples/tests`.

## LICENSE

See LICENSE.
