# homework3
- 重写rasterize_triangle，三角形内，每个pixel的颜色通过顶点的信息，用重心坐标插值得到。
- 值得注意的是坐标使用的是screen space还是camera space，一些计算，例如Blinn-Phong计算光照时需要使用变形前的场景中的坐标来计算距离和光照。
- 写几种片元着色器：分别使用纹理着色，布林-冯(Blinn-Phong)光照模型着色，法线着色，凹凸贴图着色(Bump)和它们的组合。


### Blinn-Phong光照模型

Blinn-Phong = 环境光(ambient) + 漫反射(diffuse) + 高光(specular) = La + Ld + Ls
= ka * la + kd * (I/r^2) * max(0, n·l) + ks * (I/r^2) * max (0, n·h)^p
k是系数，I是光源的intensity，r点与光源的距离，n法线，h半程向量，p也是与材质关联的系数。


### Bump
在某点，对应贴图的(u,v)处,若原先法线为(0,0,1)
dp/du = c1 * (h(u+1,v) - h(u,v)) 
dp/dv = c2 * (h(u,v+1) - h(u,v))

n = (-dp/dv, -dp/dv, 1).normalized()


