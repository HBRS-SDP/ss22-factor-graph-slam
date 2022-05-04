## Installing GNU-GSl:
- Download the latest stable version in [GNU-GSL 2.7](https://ftp.halifax.rwth-aachen.de/gnu/gsl/gsl-2.7.tar.gz)
- Create a separate workspace and extract the tar file (prefarably at your home directory)
  <br> `mkdir gsl`<br> 'cd gsl`

- Configure the workspace with required libraries for GNU-GSL by running the command in your GSL directory <br>
  `./configure --prefix = '/home/--name--/gsl/gsl-install'` <br>**NOTE: enter your home directory name instead of --name--** <br>
 - Navigate into GSL directory (i.e) `$home/--name--/gsl/gsl-install`<br>
    - run the command `make` . Once its finished running, then
    - Run the command `make install` which will install all the static and dynamic libraries inside the workspace folder

## Running a simple program

- Create a sample C program [refer here for examples](https://www.gnu.org/software/gsl/doc/html/usage.html)
- Compile it using the command <br>
  `gcc -Wall -c -I /home/selva/gsl/gsl-install/include/ <name_of_your_file>.c` it will create an intermediate object file (i.e) <name of your file.o>
- Then, run the command `gcc -L/home/--name--/gsl/gsl-install/lib/ <name_of_your_file>.o -lgsl -lgslcblas -lm`
  
  ### What's the command means?
  
    - **lgsl** - looks inside the libgsl folder
    - **Cblas** - linear algebra library for compilation
    - **lm** - links the code with C library
 - After running the above command, a.out file will be generated. Visualize the datapoints using the command `./a.out`
  
## MakeFile
  ToDo
  
## Common errors and solutions
 
- After ./a.out execution if there is an error stating `libgsl.so.25: cannot open shared object file`. Then, run the folllowing commands in the same path
  - `echo $LD_LIBRARY_PATH`<br>
  - `export LD_LIBRARY_PATH=/home/--name--/gsl/gsl-install/lib/`
- Better way to avoid the error permanently is by adding the above commands to bash.rc
