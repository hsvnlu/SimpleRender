#include"Model.h"
#include<string>
#include"OBJ_Loader.h"

void Model::setModelMatrix(const Matrix4f &Translation, const Matrix4f &Rotation, const Matrix4f &Scalar)
{
    M = Translation * Rotation * Scalar;
}

Model::Model()
{

    Triangle *tri = new Triangle;
    tri->setVertex(0, { 1.f, 0.f, -1.f, 1.f });
    tri->setVertex(1, { 0.f, 1.f, -1.f, 1.f });
    tri->setVertex(2, { -1.f, 0.f, -1.f, 1.f });
       
    tri->setNormal(0, { 0.f, 0.f, -1.f, 0.f });
    tri->setNormal(1, { 0.f, 0.f, -1.f, 0.f });
    tri->setNormal(2, { 0.f, 0.f, -1.f, 0.f });
       
    tri->setTexCoord(0, { 0.1f, 0.f });
    tri->setTexCoord(1, { 0.5f, 1.f });
    tri->setTexCoord(2, { 0.0f, 0.f });

    TriangleList.push_back(tri);
    M = Matrix4f::Identity();
    texture = Texture("hmap.jpg");
}

Model::Model(const std::string &objPath, const std::string &texPath)
{
    objl::Loader Loader;
    bool loadout = Loader.LoadFile(objPath);
    for (auto mesh : Loader.LoadedMeshes)
    {
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            Triangle* t = new Triangle();
            for (int j = 0; j < 3; j++)
            {
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0f));
                t->setNormal(j, Vector4f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z, 0.0f));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }
    this->texture = Texture(texPath);
}