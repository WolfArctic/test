#ifndef GDEBUG_H_
#define GDEBUG_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define savedata
//#define tf_xxx
extern int fd_debug_1026[10];

inline void gassert(cudaError_t err_code, const char *file, int line)
{
	if (err_code != cudaSuccess) {
		fprintf(stderr, "Error: %s %s %d\n", cudaGetErrorString(err_code), file, line);
		cudaDeviceReset();
		exit(EXIT_FAILURE);
	}
}

inline void open_debug_file(int num)
{	
/*	char debug_file_name[50] = "";
	sprintf(debug_file_name,"/home/nvidia/debug_file_%d.txt",num);

	fd_debug_1026[num] = open(debug_file_name,O_CREAT|O_RDWR,0777);
	perror("open");
	printf("_LINE_:%d,fd=%d--%s\n",__LINE__,fd_debug_1026[num],debug_file_name);
*/
}

inline void save_debug_data(const char *data,int num,long time_now)
{
/*	int len = 0;
	len = write(fd_debug_1026[num],data,strlen(data));
	printf("_LINE_:%d,len=%d\n",__LINE__,len);
i*/
}

inline void close_debug_file_own(int num)
{
//	close(fd_debug_1026[num]);
}

#define checkCudaErrors(err_code) gassert(err_code, __FILE__, __LINE__)

#define timeDiff(start, end) ((end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec)

#endif
