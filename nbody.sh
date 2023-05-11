#!/bin/bash -l
#SBATCH --job-name=output.txt
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --gpus=1
#SBATCH --output output.txt-job_%j.out
#SBATCH --error output.txt-job_%j.err
#SBATCH --partition=gpu-v100

# Start my application
srun nbody