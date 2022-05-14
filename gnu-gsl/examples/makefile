all:
	gcc -Wall -c -I /home/selva/gsl/gsl-install/include/ basic_examples.c
	gcc -L/home/selva/gsl/gsl-install/lib/ basic_examples.o -lgsl -lgslcblas -lm
	./a.out