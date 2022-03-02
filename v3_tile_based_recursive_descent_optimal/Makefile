CFLAG += -fPIC -O0 -g #-fsanitize=address
CFLAG += -lm
CFLAG += -std=c++11 -Wno-unused-result


all:
	g++ *.cpp -o raster $(CFLAG) $(IFLAG)

clean:
	rm -f *.o raster
