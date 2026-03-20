# Assignment 2.1

1. Build command:
` g++ src/timer-xenomai.cpp -I ./include -o timer-xenomai $(pkg-config /usr/evl/lib/pkgconfig/evl.pc –cflags –libs)`
2. Run command:
`sudo ./timer-xenomai`
3. Open `output.csv` to see the measurements.