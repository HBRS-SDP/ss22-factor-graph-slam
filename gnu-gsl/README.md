## Factor Graph for the GNU Scientific Library

This directory entirely contains the development files for the GNU Scientific Library. <br>
### Directory structure
- **docs** contains <br>
    - installation tutorial for GNU - GSL library (version 2.7)
    - a markdown file with functions and algorithms needed for implementing factor graphs in C (in progress)

- **examples** contains <br>
    - basic tutorials in C for the GNU - GSL library (derived from the GSL documentation)
    - to show simpler examples I added Bessel function, eigen value and vector calculation, and matrix norm calculation in a single program (i.e basic_examples.c). I added a makefile to make the compiling and running process easier.
    - Run the `make` command inside the examples directory to compile and run the c codes <br>
    ## Run through 
    - ![one](https://user-images.githubusercontent.com/91877888/168449355-7af4c3dd-f340-48a7-9979-03bdec23e84e.png)
    - ![two](https://user-images.githubusercontent.com/91877888/168449361-cec29743-1f53-4194-ab53-ecff11d8065a.png)
    - ![three](https://user-images.githubusercontent.com/91877888/168449368-4da916a0-74e6-40ea-8418-88102cab761e.png)

**NOTE: Makefile provided in the gnu-gsl directory is for compiling the dependencies and libraries for the GNU-GSL library. (PS: Don't run it)**
