#ifndef CARTOGRAPHER_IO_CAIRO_TYPES_H_
#define CARTOGRAPHER_IO_CAIRO_TYPES_H_

#include <memory>

#include "cairo/cairo.h"  
// 参考：https://blog.csdn.net/flexwang_/article/details/38000401

namespace cartographer {
namespace io {
namespace cairo {

// std::unique_ptr for Cairo surfaces. The surface is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniqueSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t*)>;

// std::unique_ptr for Cairo contexts. The context is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniqueContextPtr = std::unique_ptr<cairo_t, void (*)(cairo_t*)>;

// std::unique_ptr for Cairo paths. The path is destroyed when the
// std::unique_ptr is reset or destroyed.
using UniquePathPtr = std::unique_ptr<cairo_path_t, void (*)(cairo_path_t*)>;

}  // namespace cairo
}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_CAIRO_TYPES_H_

/*
Cairo是非常流行的开源2D图形渲染引擎库，它支持包括X-Windos，Win32，图像，pdf在内的各种输出设备。
目前，Cairo已被广泛的使用在多个平台上来渲染图形界面，包括Firefox/Webkit-EFL/GTK+/Poppler等等。 

sudo apt-get install libcairo2-dev

对于Cairo的基本绘图模型做一了解。
这些模包括：
表面（surfac），源（source)，遮盖(mask)，路径(path),上下文(context)和函数(verb)

表面(surface):
Surface是Cair绘图的目标区域，在Cairo中使用cairo_surface_t表示。
前面讲到Cair支持多种输出设备，因此我们绘图的目标区域可能是一张png图象也可能是一个pdf文件。
不同目标的绘制的底层实现各不相同，而surfac对这些绘图的目标进行了一个抽象。
因此，我们在创建了相应的surface后，只需要调用统一的函数对surface进行绘制，
而不需要关心其后端(backend)的具体实现。

源(source):
Source指的是我们绘图是的具体的材料与格式，它包括画笔的粗细、颜色等等。
在Cairo中，source不光可以是简的颜色，也可以是一种图案（patter）比如渐变色，
甚至可以是一个表面（surface)。

遮盖（mask）:
Mask相当于我们在绘图过程，用一张挖空了某些部分的纸遮挡在画布上。
这样，在绘图过程中，只有挖空的部分会被我们所使用的源影响到，其余部分不受影响。

路径（path）:
Path是指Cairo的绘制表面上一些虚拟的路径，它可能是一条线段，一个闭合的四边形，
也可能是更加复杂的曲线。Path可以由Cairo的函数（在Cairo中被称为verb）所创建。
但是，由于Path只是虚拟的路径，所以对Path的创建并不代表对表面绘制。
接下来，我们还需要使用绘制函数（Cairo里称为drawing verb)进行绘制。
比如，我们可以通过cairo_rectangle函数创建一个闭合的长方形的路径，
然后通过cairo_fill函数填充这个长方形。

上下文(context):
Context是Cairo的核心结构，在Cairo中使用cairo_t来表示。
它记录了当前状态下，与绘制有关的各种信息，包括之前介绍过的表面、源、遮盖、字体等等。 
在任何绘制之前，我们都必须先创建一个cair_t结构，同时将它绑定到一个绘制表面上(surface)。
下面的代码段创建了一个cairo_t，并将其绑定到一个640x480的png图象上。

    cairo_surface_t *surface;
    cairo_t *cr;
     
     
    surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32, 640, 480);
    cr = cairo_create (surface);

函数(verb):
Cairo中与绘制相关的函数被称为verb。目前Cairo支持五种drawing verb，分别是stroke(画线)， 
fill(填充)，text（文字），paint（滤镜），mask（遮盖）。其中，paint相当于是整个源进行了一次操作，
比如cairo_paint_with_alpha函数可以设置一个alpha值，来对整个图象进行灰度的减少。

变换(transformation):
Cairo还提供类似OpenGL的坐标变换操作。
变换操作包括：平移(cairo_translate)，伸缩(cairo_scale)，旋转(cairo_rotate)。
我们也可以通过cairo_transform函数来指定一个复杂的变换。

eg:
绘制一个矩形到rectangle.png图片上。
#include <cairo.h>
 
int main (int argc, char *argv[])
{
    cairo_surface_t *surface;
    cairo_t *cr;
 
    int width = 640;
    int height = 480;
    surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32, width, height);
    cr = cairo_create (surface);
 
    // Drawing code goes here 
    cairo_set_line_width (cr, 10);
    cairo_set_source_rgb (cr, 0, 0, 0);
    cairo_rectangle (cr, width/4, height/4, width/2, height/2);
    cairo_stroke (cr);
 
    // Write output and clean up 
    cairo_surface_write_to_png (surface, "rectangle.png");
    cairo_destroy (cr);
    cairo_surface_destroy (surface);
 
    return 0;
}


 绘制helloworld到helloworld.pdf上。
#include <cairo.h>
#include <cairo-pdf.h>
int main (int argc, char *argv[])
{
    cairo_surface_t *surface;
    cairo_t *cr;
    cairo_text_extents_t te;
 
 
   // Prepare drawing area 
    int width = 200;
    int height = 120;
      
    surface = cairo_pdf_surface_create ("helloworld.pdf", width, height);
    cr = cairo_create (surface);
 
 
    // Drawing code goes here 
    cairo_set_source_rgb (cr, 0.0, 0.0, 0.0);
    cairo_select_font_face (cr, "Georgia",
        CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size (cr, 20);
    cairo_text_extents (cr, "helloworld", &te);
    cairo_move_to (cr, width/2 - te.width / 2 - te.x_bearing,
          height/2 - te.height / 2 - te.y_bearing);
    cairo_show_text (cr, "helloworld");
 
 
    cairo_destroy (cr);
    cairo_surface_destroy (surface);
 
 
    return 0;
}
*/