# RTClib

A modern Arduino RTC library, with usability in mind and not reinventing the wheel.

> [!CAUTION]
>
> This library is still in development, documentations and examples are yet to be written, and *some* of the APIs are subject to change. Any feedback is welcome!

## Why another RTC library?

Take a search for "RTC library" in your Arduino IDE or on GitHub and you'll find a plethora of existing libraries. Yes, many of them are great, but many of them are not. Many of them are old, outdated, and not maintained. Many of them have poorly designed APIs, full of technical debt, unintuitive to use, and not following modern C++ best practices.

Yes, this is understandable - many of these libraries were written years ago, when the Arduino ecosystem was much different. Also, since Arduino is a hobbyist platform, many of these libraries were written by hobbyists. Yes, we all learn from writing code, but more has to be considered when writing a library that is to be used by others.

For example, many are aware of the [Adafruit RTClib](https://github.com/adafruit/RTClib). People at Adafruit are great, and they have done a lot for the Arduino community. But here are some points that I consider bad practices:

- **Overhead**: Adafruit RTClib is based on their BusIO library, which adds abstraction layers (read as "a lot of overhead") and is in many cases an unnegligible burden for small microcontrollers like ATmega328P.
- **Reinventing the Wheel**: Like most other (if not all) RTC libraries out there, it has its own `DateTime` and `TimeSpan` class and all sorts of other things which also add overhead and requires extra learning.
- **Design**: Inside you'll find different naming conventions (like `camelCase` and `flatcase` mixed together), different coding styles, over-encapsulation (yes I am talking about `DateTime` and `TimeSpan` again), and more.
- **Limited Support**: It's shocking (at least for me) to see that many libraries out there does not support DS1302, which, though not as accurate, is still a popular RTC chip.

# So what's the difference?

The development of this library started when NeiroN, author of another RTC library, deleted their GitHub account and all of their repositories. I (@45gfg9) was a maintainer of that library (which is still accessible in the [PlatformIO library registry](https://registry.platformio.org/libraries/neironx/RTCLib)), which I had to admit was not well-designed. So I decided to start from scratch, with the following principles in mind:

- **No dependencies**: It is a standalone library that only depends on the Arduino core and standard C/C++ libraries that are available even in avr-libc.
- **Modern C/C++**: Written in C++11 with Arduino ecosystem in mind. Inside you'll find good use of iterators, RAII, and more.
- **Minimal Overhead**: The library is designed to be as lightweight as possible. Optimizations are made to a certain extent where usability and readability are not compromised.
- **No Reinventing the Wheel**: You'll find no auxiliary classes, no `DateTime` class, no `Time` class, no `TimeSpan` class, etc. We already have standardized `time_t` and `tm`. We already have `localtime`, `mktime`, `asctime` and more. So why reinvent the wheel when learning to cope with the standard is (almost certainly) a better investment in the long run?

## But there is a catch here, isn't it?

... yes. First, this library is not designed to be a drop-in replacement for other libraries, though it would be easy to port your code to this library. Second, to maintain minimal runtime overhead, there's no OOP features like inheritance and polymorphism. This is a design choice, and it is not likely to change. We're relying on compiler optimizations to inline functions and cut off code that are not used in your sketch. Third, due to the complex nature of different RTC chips having different register maps and relying on different bits & bytes magic, it's hard to balance API cleanness and feature completeness. If you have a better idea and/or is willing to contribute, please do so!

# Try it out!

Since the library is still in development, it is not yet available in the Arduino Library Manager. You will need to download / `git clone` the source code and install it manually. PlatformIO users will need to add the full URL to your `platformio.ini` file.

> [!WARNING]
>
> Examples are yet to be written. Please stay tuned!

# License

This library is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.
