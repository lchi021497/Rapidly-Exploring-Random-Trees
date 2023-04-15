app: rrt.o
	g++ rrt.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system

rrt.o:
	g++ -std=c++17 -I/usr/local/include geometry.h -c rrt.cpp

.PHONY: clean
clean:
	rm *.o sfml-app
