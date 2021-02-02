ikd_Tree_demo : ikd_Tree_demo.o ikd_Tree.o 
	g++ -std=c++11 -Wall ikd_Tree_demo.o ikd_Tree.o -o ikd_Tree_demo -pthread

ikd_Tree_demo.o : ikd_Tree_demo.cpp
	g++ -c ikd_Tree_demo.cpp 

ikd_Tree.o : ikd_Tree.cpp ikd_Tree.h
	g++ -c ikd_Tree.cpp 

clean:
	rm *.o ikd_Tree_demo