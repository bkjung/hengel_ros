#include <flann/flann.hpp>
#include <flann/flann.h>

#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#define FLANN_USE_CUDA


float* read_points(const char* filename, int rows, int cols)
{
    float* data;
    float *p;
    FILE* fin;
    int i,j;

    fin = fopen(filename,"r");
    if (!fin) {
        printf("Cannot open input file.\n");
        exit(1);
    }

    data = (float*) malloc(rows*cols*sizeof(float));
    if (!data) {
        printf("Cannot allocate memory.\n");
        exit(1);
    }
    p = data;

    for (i=0;i<rows;++i) {
        for (j=0;j<cols;++j) {
            fscanf(fin,"%g ",p);
            p++;
        }
    }

    fclose(fin);

    return data;
}

void write_results(const char* filename, int *data, int rows, int cols)
{
    FILE* fout;
    int* p;
    int i,j;

    fout = fopen(filename,"w");
    if (!fout) {
        printf("Cannot open output file.\n");
        exit(1);
    }

    p = data;
    for (i=0;i<rows;++i) {
        for (j=0;j<cols;++j) {
            fprintf(fout,"%d ",*p);
            p++;
        }
        fprintf(fout,"\n");
    }
    fclose(fout);
}



int main(int argc, char** argv)
{
    clock_t tStart = clock();

    float* dataset;
    float* testset;
    int nn;
    int* result;
    float* dists;
    struct FLANNParameters p;
    float speedup;
    flann_index_t index_id;

    int rows = 9000;
    int cols = 128;
    int tcount = 1000;

    /*
     * The files dataset.dat and testset.dat can be downloaded from:
     * http://people.cs.ubc.ca/~mariusm/uploads/FLANN/datasets/dataset.dat
     * http://people.cs.ubc.ca/~mariusm/uploads/FLANN/datasets/testset.dat
     */
    printf("Reading input data file.%s\n", argv[1]);
    dataset = read_points(argv[1], rows, cols);
    printf("Reading input data file.%s\n", argv[2]);
    testset = read_points(argv[2], tcount, cols);

    nn = 3;
    result = (int*) malloc(tcount*nn*sizeof(int));
    dists = (float*) malloc(tcount*nn*sizeof(float));

    p = DEFAULT_FLANN_PARAMETERS;
    p.algorithm = FLANN_INDEX_KDTREE;
    p.trees = 8;
    p.log_level = FLANN_LOG_INFO;
    p.checks = 64;

    printf("Computing index.\n");
    index_id = flann_build_index(dataset, rows, cols, &speedup, &p);
    flann_find_nearest_neighbors_index(index_id, testset, tcount, result, dists, nn, &p);

    write_results("results.dat",result, tcount, nn);

    flann_free_index(index_id, &p);
    free(dataset);
    free(testset);
    free(result);
    free(dists);

    printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    return 0;
}


//int main(int argc, char** argv)
//{
//    float* gpu_pointer = (float*)
//}
//
