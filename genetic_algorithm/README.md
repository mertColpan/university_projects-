# Genetic Algorithm Experimentation

## Project Overview

This project is a Python application that visualizes the performance of genetic algorithms under various parameter combinations. Experiments are conducted by altering parameters such as population size, mutation rate, selection method, and crossover method, allowing you to observe how genetic algorithms perform with different configurations. The genetic algorithm attempts to match 24x24 binary images using 3x3 blocks of 7 patterns per gene, aiming to evolve the population towards a better solution over multiple generations.

The project uses these parameters to adjust the behavior of the genetic algorithm and analyze its effectiveness in solving the image matching problem.


## Features

- **Population Size**: Experiments are conducted with varying population sizes.
- **Mutation Rate**: Trials are conducted with different mutation rates.
- **Selection Method**: The algorithm allows experimenting with different selection strategies (e.g., elitism selection).
- **Crossover Method**: Different crossover methods (e.g., one-point crossover) are tested.
- **Visualization**: The fitness values over time for each parameter combination are visualized.

## Getting Started

### Requirements

To run this project, Python and some libraries are required. You can install the necessary libraries with the following command:

`pip install numpy matplotlib os argparse`

### Cloning Repository
```bash
git clone https://github.com/yourusername/genetic-algorithm-visualizer.git`
```


## Available Arguments:
    


**--selection_method:** Select the selection method. Options are roulette or tournament. Default: roulette.

**--crossover_method:** Choose the crossover method. Options are single_point, multi_point, or uniform. Default: multi_point.

**--mutation_rates:** Specify the mutation rate as a float value. Default: 0.1.

**--populations:** Set the population size as an integer. Default: 20.

**--generation_num:** Set the number of generations. Default: 30.

**--repetitions:** Specify how many times to repeat the experiment to ensure statistical robustness. Default: 1.

**--image_number:** Specify the image number (0, 1, or 2). Default: 0.


## Example Usage
```bash
python3 main.py --populations 5 10 15 --mutation_rates 0.2 0.1 --generation_num 25 --repetitions 100 --selection_method elitism_selection --crossover_method multi_point  --generation_num_exp 30 --image_number 1
```


