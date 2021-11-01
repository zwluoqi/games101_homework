1.光线生成完成
2.正确实现了 Moller-Trumbore 算法完成光线与三角形相交
3.global.hpp中solveQuadratic函数为处理a为零的情况，添加额外判定：
x0 = x1 = -0.5 * b / a;

4.Render.cpp中Render函数未中unsigned char color的强转问题：
color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
