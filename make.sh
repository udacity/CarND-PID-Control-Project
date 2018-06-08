echo "Compiling main.cpp"
g++ -o build/main.out src/main.cpp src/PID.cpp src/PID.h -std=c++11 /usr/lib/libuWS.so /usr/lib/x86_64-linux-gnu/libssl.so /usr/lib/x86_64-linux-gnu/libz.so
