#include"Rasterizer.h"
Vector3f shadeFragment(const Triangle& tri, const Texture& texture, const std::vector<Light> lights, float alpha, float beta, float gamma, float weight, const Triangle &viewpos);

void Rasterizer::rendering(const Model model, const std::vector<Light> &lights, const Transformation& trans) {
	int aabbxmin, aabbymin, aabbxmax, aabbymax;
	float t1, t2, t3, alpha, beta, gamma, u, v, weight;
	//MVP变换，变换光源和模型， 三角形顶点的法线要特殊处理
	Matrix4f mvp = trans.P * trans.V * model.M;
	Matrix4f mv = trans.V * model.M;
	Matrix4f mvForNormal = (trans.V * model.M).inverse().transpose();
	//对光源变换
	std::vector<Light> viewLight = lights;
	for (auto &light : viewLight) {
		light.position = trans.V * light.position;
	}

	//开始逐个渲染三角形
	Triangle rtTri, viewpos;
	for (const auto &ele : model.TriangleList) {
		//变换rtTri得到光栅化三角形
		rtTri = *ele;
		viewpos = *ele;
		for (int i = 0; i < 3; ++i) {
			rtTri.vertex[i] = mvp * rtTri.vertex[i];
			viewpos.vertex[i] = mv * viewpos.vertex[i];
			float w = rtTri.vertex[i].w();
			rtTri.vertex[i].x() /= w;
			rtTri.vertex[i].y() /= w;
			rtTri.vertex[i].z() /= w;
			rtTri.vertexNormal[i] = mvForNormal * rtTri.vertexNormal[i];
		}
		//viewpoint变换
		for (auto& v : rtTri.vertex) {
			v.x() = (v.x() + 1.0f) * width * 0.5f;
			v.y() = (v.y() + 1.0f) * height * 0.5f;
		}

		//判断哪些点在三角形内，并渲染
		aabbxmin =  ceil(std::min(rtTri.vertex[0].x(), std::min(rtTri.vertex[1].x(), rtTri.vertex[2].x())));
		aabbymin =  ceil(std::min(rtTri.vertex[0].y(), std::min(rtTri.vertex[1].y(), rtTri.vertex[2].y())));
		aabbxmax = floor(std::max(rtTri.vertex[0].x(), std::max(rtTri.vertex[1].x(), rtTri.vertex[2].x())));
		aabbymax = floor(std::max(rtTri.vertex[0].y(), std::max(rtTri.vertex[1].y(), rtTri.vertex[2].y())));
		for (float x = aabbxmin; x <= aabbxmax && x <= width; ++x) {
			for (float y = aabbymin; y <= aabbymax && y <= height; ++y) {
				t1 = (rtTri.vertex[1].x() - rtTri.vertex[0].x()) * (y - rtTri.vertex[0].y()) - (x - rtTri.vertex[0].x()) * (rtTri.vertex[1].y() - rtTri.vertex[0].y());
				t2 = (rtTri.vertex[2].x() - rtTri.vertex[1].x()) * (y - rtTri.vertex[1].y()) - (x - rtTri.vertex[1].x()) * (rtTri.vertex[2].y() - rtTri.vertex[1].y());
				t3 = (rtTri.vertex[0].x() - rtTri.vertex[2].x()) * (y - rtTri.vertex[2].y()) - (x - rtTri.vertex[2].x()) * (rtTri.vertex[0].y() - rtTri.vertex[2].y());
				if (((t1 > 0) && (t2 > 0) && (t3 > 0)) || ((t1 < 0) && (t2 < 0) && (t3 < 0))) {
					alpha = ((rtTri.vertex[1].x() - x) * (rtTri.vertex[2].y() - y) - (rtTri.vertex[2].x() - x) * (rtTri.vertex[1].y() - y)) / ((rtTri.vertex[1].x() - rtTri.vertex[0].x()) * (rtTri.vertex[2].y() - rtTri.vertex[0].y()) - (rtTri.vertex[2].x() - rtTri.vertex[0].x()) * (rtTri.vertex[1].y() - rtTri.vertex[0].y()));
					beta  = ((rtTri.vertex[0].x() - x) * (rtTri.vertex[2].y() - y) - (rtTri.vertex[2].x() - x) * (rtTri.vertex[0].y() - y)) / ((rtTri.vertex[0].x() - rtTri.vertex[1].x()) * (rtTri.vertex[2].y() - rtTri.vertex[1].y()) - (rtTri.vertex[2].x() - rtTri.vertex[1].x()) * (rtTri.vertex[0].y() - rtTri.vertex[1].y()));
					gamma = 1 - alpha - beta;
					weight = 1.0f / (alpha / rtTri.vertex[0].w() + beta / rtTri.vertex[1].w() + gamma / rtTri.vertex[2].w());
					float Z = weight * (alpha * rtTri.vertex[0].z() / rtTri.vertex[0].w() + beta * rtTri.vertex[1].z() / rtTri.vertex[1].w() + gamma * rtTri.vertex[2].z() / rtTri.vertex[2].w());
					if (Z > depth_buffer[getBufferIndex(x, y)]) {
						depth_buffer[getBufferIndex(x, y)] = Z;
						frame_buffer[getBufferIndex(x, y)] = shadeFragment(rtTri, model.texture, viewLight, alpha, beta, gamma, weight, viewpos);
					}
				}
			}
		}
	}
}

size_t Rasterizer::getBufferIndex(float x, float y) const
{
	return size_t(floor(x) + (height - floor(y)) * width);
}

Vector4f interpolate(const Vector4f* attribt, const Vector4f* triPos, float alpha, float beta, float gamma, float weight) {
	return (alpha * attribt[0] / triPos[0].w() + beta * attribt[1] / triPos[1].w() + gamma * attribt[2] / triPos[2].w()) * weight;
}

Vector3f interpolate(const Vector3f *attribt, const Vector4f* triPos, float alpha, float beta, float gamma, float weight) {
	return (alpha * attribt[0] / triPos[0].w() + beta * attribt[1] / triPos[1].w() + gamma * attribt[2] / triPos[2].w()) * weight;
}

Vector2f interpolate(const Vector2f* attribt, const Vector4f* triPos, float alpha, float beta, float gamma, float weight) {
	return (alpha * attribt[0] / triPos[0].w() + beta * attribt[1] / triPos[1].w() + gamma * attribt[2] / triPos[2].w()) * weight;
}


static Vector3f BlinnPhongShading(const Vector4f &pos, const Vector3f &texColor, const Vector3f &normal, const std::vector<Light> &lights)
{
	Vector3f ka = Vector3f(0.005, 0.005, 0.005);
	Vector3f kd = texColor / 255.f;
	Vector3f ks = Vector3f(0.500, 0.500, 0.500);
	Vector3f amb_light_intensity{ 10, 10, 10 };

	float p = 200;

	Vector3f point = pos.head(3);

	Vector3f result_color = { 0, 0, 0 };
	for (auto& light : lights)
	{
		float r2 = (light.position.head(3) - point).squaredNorm();
		Vector3f l = (light.position.head(3) - point).normalized();
		Vector3f n = normal.normalized();
		Vector3f v = (- point).normalized();
		Vector3f h = (l + v).normalized();
		//amb
		result_color += ka.cwiseProduct(amb_light_intensity);
		//diffuse
		result_color += (kd.cwiseProduct(light.intensity / r2)) * std::max(0.0f, n.dot(l));
		//specular
		result_color += (ks.cwiseProduct(light.intensity / r2)) * std::pow(std::max(0.f, (n.dot(h))), p);
	}

	return result_color * 255.f;
}

//bump
Vector3f BumpShading(const Vector4f& pos, const Vector3f& texColor, const Vector2f& texCoords, const Vector3f& normal, const std::vector<Light>& lights, const Texture &texture)
{	float kh = 0.2, kn = 0.1;

	Vector3f n = normal;
	Vector3f t = { n.x() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()), sqrt(n.x() * n.x() + n.z() * n.z()), n.z() * n.y() / sqrt(n.x() * n.x() + n.z() * n.z()) };
	Vector3f b = n.cross(t);
	float u, v, w, h;
	u = texCoords[0];
	v = texCoords[1];
	w = texture.width;
	h = texture.height;

	float dU = kh * kn * (texture.getTexColor(u + 1 / w, v).norm() - texture.getTexColor(u, v).norm());
	float dV = kh * kn * (texture.getTexColor(u, v + 1 / h).norm() - texture.getTexColor(u, v).norm());
	Vector3f ln(-dU, -dV, 1);
	Matrix3f TBN;
	TBN.col(0) = t;
	TBN.col(1) = b;
	TBN.col(2) = n;

	Vector3f result_color = { 0, 0, 0 };
	result_color = (TBN * ln).normalized();

	return result_color;
}

Vector3f shadeFragment(const Triangle& tri, const Texture& texture, const std::vector<Light> lights, float alpha, float beta, float gamma, float weight, const Triangle& viewpos) {
	Vector4f pos = interpolate(viewpos.vertex, tri.vertex, alpha, beta, gamma, weight);
	Vector2f texCoord = interpolate(tri.texCoord, tri.vertex, alpha, beta, gamma, weight);
	Vector3f texColor = texture.getTexColor(texCoord.x(), texCoord.y());
	Vector3f normal = interpolate(tri.vertexNormal, tri.vertex, alpha, beta, gamma, weight).head(3).normalized();
	normal = BumpShading(pos, texColor, texCoord, normal, lights, texture);
	return BlinnPhongShading(pos, texColor, normal, lights);
}
