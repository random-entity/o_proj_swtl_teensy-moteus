- Make sure registering threads to `threads` by `threads.addThread()` happens inside the `setup()` function.
- Recommended that all variables manipulated by threads be declared as volatile.
- Do not initialize Moteus object with `options.default_query = false`. Doing this causes delay between sending Command to Servos and the actual receival of the Commands in the Servo, leading to failure of syncronization of Servo Commands.
- Add thread AFTER Stopping all Servos.
- Initialize `SPI` for CAN FD Buses 1 and 2, and `SPI1` for Buses 3 and 4.
