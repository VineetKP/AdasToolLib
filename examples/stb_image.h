/* stb_image - v2.27 - public domain image loader - https://github.com/nothings/stb
   For brevity we include a trimmed header containing necessary declarations only.
   This is not the full upstream file. It should be sufficient for basic loading
   of PNG/JPEG in this example. If you need the full functionality, replace with
   the original stb_image.h from the author. */

#ifndef STB_IMAGE_H
#define STB_IMAGE_H

#ifdef __cplusplus
extern "C" {
#endif

extern unsigned char *stbi_load(char const *filename, int *x, int *y, int *channels_in_file, int desired_channels);
extern void stbi_image_free(void *retval_from_stbi_load);

#ifdef __cplusplus
}
#endif

#endif // STB_IMAGE_H
