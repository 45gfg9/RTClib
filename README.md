# RTClib

A modern Arduino RTC library, with usability in mind and not reinventing the wheel.

> [!CAUTION]
>
> This library is still in development, documentations and examples are yet to be written, and *some* of the APIs are subject to change. Any feedback is welcome!

## Why another RTC library?

Take a search for "RTC library" in your Arduino IDE or on GitHub and you'll find a plethora of existing libraries. Yes, many of them are great, but many of them are not. Many of them are old, outdated, and not maintained. Many of them have poorly designed APIs, full of technical debt, unintuitive to use, and not following modern C++ best practices.

Yes, this is understandable - many of these libraries were written years ago, when the Arduino ecosystem was much different. Also, since Arduino is a hobbyist platform, many of these libraries were written by hobbyists. Yes, we all learn from writing code, but more has to be considered when dedicating a library to the public.

For example, [Adafruit RTClib](https://github.com/adafruit/RTClib) is a fantastic library. People at Adafruit are great, and they have undoubtedly done a lot for the Arduino community. But here are some points that I consider bad practices:

- **Overhead**: Adafruit RTClib is based on their BusIO library, which adds abstraction layers (read as "a lot of overhead") introduced by polymorphism. It also uses dynamic memory allocation that adds extra heap-management code to the final ELF. These in many cases are non-negligible burdens, especially for resource-constrained MCUs like the ATmega328P.
- **Reinventing the Wheel**: Like most other (if not all) RTC libraries out there, it ships with its own `DateTime` and `TimeSpan` class and all sorts of other things which also contribute to overhead and learning curve.
- **Design**: Inside you'll find different naming conventions (like `camelCase` and `flatcase` mixed together), different coding styles, over-encapsulation (yes I am talking about `DateTime` and `TimeSpan` again), and more.
- **Limited Support**: It's shocking (at least for me) to see that many libraries out there does not support DS1302, which, though not as accurate, is still a widely-used (and cheap!) RTC chip.

# So what's all these fuss about?

The development of this library started when NeiroN, author of another RTC library, deleted their GitHub account and all of their repositories. I (@45gfg9) was a maintainer of that library (which is still accessible in the [PlatformIO library registry](https://registry.platformio.org/libraries/neironx/RTCLib)), which I had to admit was not well-designed. So I decided to start from scratch, with the following principles in mind:

- **No dependencies**: It is a standalone library that only depends on the Arduino core and standard C++ libraries that are available even in avr-libc.
- **Modern C++**: Written in C++11 with Arduino ecosystem in mind. Inside you'll find good use of iterators, RAII, and more. (Well, not *that* modern since avr-libc lacks all the fancy containers and algorithms, but still better than the old C-style code you'll find in many libraries.)
- **Minimal Overhead**: The library is designed to be as lightweight as possible. Optimizations are made to a certain extent where usability and readability are not compromised.
- **No Reinventing the Wheel**: You'll find no auxiliary classes, no `DateTime` class, no `TimeSpan` class, etc. We already have standard [`time_t`](https://en.cppreference.com/w/cpp/chrono/c/time_t) and [`tm`](https://en.cppreference.com/w/cpp/chrono/c/tm). We already have [`localtime`](https://en.cppreference.com/w/cpp/chrono/c/localtime), [`mktime`](https://en.cppreference.com/w/cpp/chrono/c/mktime), [`asctime`](https://en.cppreference.com/w/cpp/chrono/c/asctime), [`difftime`](https://en.cppreference.com/w/cpp/chrono/c/difftime) and more (and yes, [avr-libc has them shipped](https://avr-libc.nongnu.org/user-manual/group__avr__time.html)). So why implement another class when learning to cope with the standard is (almost certainly) a better investment in the long run?

## But there's a catch, isn't it?

... yes, but for good reasons. First, this library is not designed to be a drop-in replacement for other libraries, though it would be easy to port your code to this library. Second, to maintain minimal runtime overhead, there's no OOP features like inheritance and polymorphism. This is a design choice, and it is not likely to change. We're relying on compiler optimizations to inline function calls and strip unused code. Third, due to the complex nature of different RTC chips having different register maps, different feature sets and different bits & bytes magic, it's hard to balance API cleanness and feature completeness. After all, the greatest common ground of all RTC chips is that they all have a clock and can keep track of time! I'm trying my best not to make the API too complex, and if you have a better idea and/or is interested in contributing, please do so!

# Try it out!

Since the library is still in development, it is not yet available in the Arduino Library Manager. You will need to download / `git clone` the source code and install it manually. PlatformIO users will need to add the full URL to your `platformio.ini` file.

> [!WARNING]
>
> Examples are yet to be written. Please stay tuned!

# License

This library is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.
