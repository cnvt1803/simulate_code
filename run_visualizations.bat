@echo off
title Coverage Path Planning Algorithms
echo ========================================
echo   Coverage Path Planning Visualizations
echo ========================================
echo.

cd /d "d:\Ho Chi Minh University of Technology\Final Project\code"

:menu
echo Available Options:
echo.
echo 1. BA* Algorithm (Pygame)
echo 2. C* Algorithm (Pygame) 
echo 3. Algorithm Comparison (Side-by-side)
echo 4. Main Menu Interface
echo 5. Test All Systems
echo 6. Exit
echo.
set /p choice="Select option (1-6): "

if "%choice%"=="1" (
    echo Starting BA* Algorithm...
    python ba_star_pygame_visualization.py
    goto menu
)
if "%choice%"=="2" (
    echo Starting C* Algorithm...
    python c_star_pygame_visualization.py
    goto menu
)
if "%choice%"=="3" (
    echo Starting Algorithm Comparison...
    python algorithm_comparison.py
    goto menu
)
if "%choice%"=="4" (
    echo Starting Main Menu...
    python main_runner.py
    goto menu
)
if "%choice%"=="5" (
    echo Running Tests...
    python test_algorithms.py
    pause
    goto menu
)
if "%choice%"=="6" (
    echo Goodbye!
    exit
)

echo Invalid choice. Please try again.
goto menu