#include <stdlib.h>
#include <math.h>
#include "vector.h"
#include "config.h"
#include <cuda_runtime.h>

__global__ void parallel_compute(vector3 *hPos, vector3 *accels, double *mass)
{
    int column = (blockDim.x * blockIdx.x) + threadIdx.x;
    int row = (blockDim.y * blockIdx.y) + threadIdx.y;
    int ind = (NUMENTITIES * row) + column;
    int i = row;
    int j = column;
    if (ind < NUMENTITIES * NUMENTITIES)
    {
        if (i == j){
        	FILL_VECTOR(accels[ind], 0, 0, 0);
        }
        else{
            vector3 distance;
            for (int k = 0; k < 3; k++){
                distance[k] = hPos[i][k] - hPos[j][k];
            }
            double magnitude_sq = distance[0] * distance[0] + distance[1] * distance[1] + distance[2] * distance[2];
            double magnitude = sqrt(magnitude_sq);
            double accelmag = -1 * GRAV_CONSTANT * mass[j] / magnitude_sq;
            FILL_VECTOR(accels[ind], accelmag * distance[0] / magnitude, accelmag * distance[1] / magnitude, accelmag * distance[2] / magnitude);
        }
    }
}

__global__ void parallel_sum(vector3 *accels, vector3 *accel_sum, vector3 *hPos, vector3 *hVel)
{
    int row = blockIdx.x * blockDim.x + threadIdx.x;
    int i = row;
    if (i < NUMENTITIES){
    	FILL_VECTOR(accel_sum[i], 0, 0, 0);
        for (int j = 0; j < NUMENTITIES; j++){
            for (int k = 0; k < 3; k++){
                accel_sum[i][k] += accels[(i * NUMENTITIES) + j][k];
            }
        }
        for (int k = 0; k < 3; k++){
            hVel[i][k] += accel_sum[i][k] * INTERVAL;
            hPos[i][k] = hVel[i][k] * INTERVAL;
        }
    }
}

// compute: Updates the positions and locations of the objects in the system based on gravity.
// Parameters: None
// Returns: None
// Side Effect: Modifies the hPos and hVel arrays with the new positions and accelerations after 1 INTERVAL
void compute()
{
    vector3 *dev_hPos, *dev_hVel, *dev_acc, *dev_sum;
    double *dev_mass;
    int blocksD = ceilf(NUMENTITIES / 16.0f);
    int threadsD = ceilf(NUMENTITIES / (float)blocksD);
    dim3 gridDim(blocksD, blocksD, 1);
    dim3 blockDim(threadsD, threadsD, 1);
    cudaMalloc(&dev_hPos,sizeof(vector3) * NUMENTITIES);
	cudaMemcpy(dev_hPos,hPos,sizeof(vector3) * NUMENTITIES,cudaMemcpyHostToDevice);
    cudaMalloc(&dev_hVel,sizeof(vector3) * NUMENTITIES);
	cudaMemcpy(dev_hVel,hVel,sizeof(vector3) * NUMENTITIES,cudaMemcpyHostToDevice);
    cudaMalloc(&dev_acc,sizeof(vector3) * NUMENTITIES);
    cudaMalloc(&dev_sum,sizeof(vector3) * NUMENTITIES);
    cudaMalloc(&dev_mass,sizeof(double) * NUMENTITIES);
    cudaMemcpy(dev_mass,mass,sizeof(double) * NUMENTITIES,cudaMemcpyHostToDevice);
    Parallel_Computation<<<gridDim, blockDim>>>(dev_hPos, dev_acc, dev_mass);
    cudaDeviceSynchronize();
    Parallel_Sum<<<gridDim.x, blockDim.x>>>(dev_acc, dev_sum, dev_hPos, dev_hVel);
    cudaDeviceSynchronize();
    cudaMemcpy(hPos,dev_hPos,sizeof(vector3) * NUMENTITIES,cudaMemcpyDeviceToHost);
    cudaMemcpy(hVel,dev_hVel,sizeof(vector3) * NUMENTITIES,cudaMemcpyDeviceToHost);
    cudaFree(dev_hPos);
    cudaFree(dev_hVel);
    cudaFree(dev_mass);
    cudaFree(dev_acc);
}
