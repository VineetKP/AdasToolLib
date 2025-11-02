/* stb_image_write - public domain - trimmed for example
   Minimal declarations used by the example. Replace with upstream file for full support. */

#ifndef STB_IMAGE_WRITE_H
#define STB_IMAGE_WRITE_H

#ifdef __cplusplus
extern "C" {
#endif

extern int stbi_write_png(char const *filename, int w, int h, int comp, const void *data, int stride_in_bytes);

#ifdef __cplusplus
}
#endif

#endif // STB_IMAGE_WRITE_H
