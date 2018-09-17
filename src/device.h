#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <signal.h>
#include <linux/videodev2.h>


#define FORMAT_UNKNOWN		-1
#define STANDARD_UNKNOWN	V4L2_STD_UNKNOWN


struct hw_buffer_t {
	void	*start;
	size_t	length;
};

struct picture_t {
	unsigned char	*data;
	unsigned long	size;
};

struct device_runtime_t {
	int					fd;
	unsigned			width;
	unsigned			height;
	unsigned			format;
	unsigned			n_buffers;
	struct hw_buffer_t	*hw_buffers;
	struct picture_t	*pictures;
	unsigned			max_picture_size;
	bool				capturing;
};

struct device_t {
	char			*path;
	unsigned		width;
	unsigned		height;
	unsigned		format;
	v4l2_std_id		standard;
	bool			dv_timings;
	unsigned		n_buffers;
	unsigned		every_frame;
	unsigned		min_frame_size;
	unsigned		jpeg_quality;
	unsigned		timeout;
	unsigned		error_timeout;

	struct device_runtime_t *run;
	sig_atomic_t volatile stop;
};


struct device_t *device_init();
void device_destroy(struct device_t *dev);

int device_parse_format(const char *const str);
v4l2_std_id device_parse_standard(const char *const str);

int device_open(struct device_t *dev);
void device_close(struct device_t *dev);
