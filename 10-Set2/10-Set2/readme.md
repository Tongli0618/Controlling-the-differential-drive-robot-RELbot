# Preparation
1. Connect to the Raspberry Pi using Remote-SSH in Visual Studio Code.
2. Upload the contents of this folder to a desired folder on the Raspberry Pi.

# POSIX Version (Part 1)
This section explains how to run the POSIX version of the code, how to change the computational load, and how to run with additional processor load.
The results will be written to `Ass2_Posix_[LOAD].txt` where `[LOAD]` is the configured computation load.

## Running
To test the timing of the POSIX version perform these steps:
1. Open `posix_version.cpp`
2. Press F5 or click the run button in the top right corner.

## Changing computational load
To run the progam with a different computational load in each timer period perform these steps:
1. Change the define directive `LOAD` on line 12 to a desired value.
2. Press F5 or click the run button in the top right corner

## Executing with additional processor load
To add extra computational load from another program perform these steps:
1. Open a new SSH terminal session
2. Execute
    ```
    sudo stress -c 4 --timeout 60
    ```
3. Return to the Remote-SSH session in Visual Studio Code
4. Press F5 or click the run button in the top right corner

# EVL Version (Part 2)
This section explains how to run the EVL version of the code on the EVL core, how to change the computational load and how to run with additional processor load. The results will be written to `Ass2_EVL_[LOAD].txt` where `[LOAD]` is the configured computation load.

## Running
To test the timing of the EVL version perform these steps:
1. Open a new SSH terminal session
2. Navigate to the folder containing `evl_version.cpp`
3. Compile the code:
    ```
    g++ evl_version.cpp -o evl_version $(pkg-config /usr/evl/lib/pkgconfig/evl.pc --cflags --libs)
    ```
4. Execute the compiled program on the EVL Core:
    ```
    sudo taskset -c 1 ./evl_version
    ```

## Changing computational load
To run the program with a different computational load in each timer period perform these steps:
1. Open `evl_version.cpp`
2. Change the define directive `LOAD` on line 15 to a desired value.
3. Navigate to the folder containing `evl_version.cpp`
4. Compile the code: 
    ```
    g++ evl_version.cpp -o evl_version $(pkg-config /usr/evl/lib/pkgconfig/evl.pc --cflags --libs)
    ```
5. Execute the compiled program on the EVL Core: 
    ```
    sudo taskset -c 1 ./evl_version
    ```

## Executing with additional processor load
To add extra computational load from another program perform these steps:
1. Execute 
    ```
    sudo stress -c 4 --timeout 60
    ```
3. Open another terminal
4. Navigate to the folder containing `evl_version.cpp`
5. Compile the code: 
    ```
    g++ evl_version.cpp -o evl_version $(pkg-config /usr/evl/lib/pkgconfig/evl.pc --cflags --libs)
    ```
6. Execute the compiled program on the EVL Core:
    ```
    sudo taskset -c 1 ./evl_version
    ```

## Plot the jitter
1. Open timing_plot.m
2. Change the name you want to compare based on the previous output name, for example:
    fileA = "Ass2_EVL_Stress_10000.txt";
    fileB = "Ass2_EVL_10000.txt";