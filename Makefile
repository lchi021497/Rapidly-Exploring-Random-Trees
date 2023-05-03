app: rrt.o
	/opt/homebrew/Cellar/llvm/16.0.2/bin/clang++ -L/usr/local/lib -fopenmp -I/opt/homebrew/opt/llvm/include -I/opt/homebrew/opt/libomp/include rrt.o kdtree.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system 

kd-tree-test: kdtree.o kdtree-test.cpp
	/opt/homebrew/Cellar/llvm/16.0.2/bin/clang++ -std=c++11 -L/usr/local/lib -g kdtree.o -o kd-tree-test kdtree-test.cpp

kdtree.o: kdtree.cpp kdtree.hpp
	/opt/homebrew/Cellar/llvm/16.0.2/bin/clang++ -std=c++11 -L/usr/local/lib -I/opt/homebrew/opt/llvm/include -I/opt/homebrew/opt/libomp/include -g -c kdtree.cpp

rrt.o: kdtree.hpp kdtree.cpp 
	/opt/homebrew/Cellar/llvm/16.0.2/bin/clang++ -g -pg -L/usr/local/lib -I/opt/homebrew/opt/llvm/include -I/opt/homebrew/opt/libomp/include -I/usr/local/include geometry.h -c rrt.cpp kdtree.hpp kdtree.cpp -fopenmp 

.PHONY: clean
clean:
	rm *.o sfml-app
clean-test:
	rm kdtree.o rm kd-tree-test
