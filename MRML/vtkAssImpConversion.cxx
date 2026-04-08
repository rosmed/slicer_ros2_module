#include "vtkAssImpConversion.h"

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/config.h>

#include <iostream>

vtkSmartPointer<vtkPolyData> vtkAssImpConversion::vtkAssImpToPolyData(const std::string& filename)
{
  Assimp::Importer importer;
  // URDF meshes are expected in their native coordinate frame.
  // Prevent Assimp from rotating Collada/DAE meshes to convert
  // from Z_UP to Y_UP, which would introduce a 90-degree rotation.
  importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
  const aiScene* scene = importer.ReadFile(filename,
                                           aiProcess_Triangulate |
                                           aiProcess_GenNormals |
                                           aiProcess_JoinIdenticalVertices |
                                           aiProcess_PreTransformVertices);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    std::cerr << "Assimp failed to read file: " << filename << " (" << importer.GetErrorString() << ")" << std::endl;
    return nullptr;
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();

  normals->SetNumberOfComponents(3);
  normals->SetName("Normals");

  vtkIdType vertexOffset = 0;

  for (unsigned int meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
    aiMesh* mesh = scene->mMeshes[meshIndex];
    if (!mesh) {
      continue;
    }

    const vtkIdType meshVertexOffset = vertexOffset;

    for (unsigned int vertexIndex = 0; vertexIndex < mesh->mNumVertices; ++vertexIndex) {
      const aiVector3D& pos = mesh->mVertices[vertexIndex];
      points->InsertNextPoint(pos.x, pos.y, pos.z);

      if (mesh->HasNormals()) {
        const aiVector3D& n = mesh->mNormals[vertexIndex];
        normals->InsertNextTuple3(n.x, n.y, n.z);
      }
    }

    for (unsigned int faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex) {
      const aiFace& face = mesh->mFaces[faceIndex];
      if (face.mNumIndices == 3) {
        vtkIdType pts[3] = {
          meshVertexOffset + face.mIndices[0],
          meshVertexOffset + face.mIndices[1],
          meshVertexOffset + face.mIndices[2]
        };
        polys->InsertNextCell(3, pts);
      }
    }

    vertexOffset += mesh->mNumVertices;
  }

  polyData->SetPoints(points);
  polyData->SetPolys(polys);
  if (polyData->GetNumberOfPoints() == normals->GetNumberOfTuples()) {
    polyData->GetPointData()->SetNormals(normals);
  }

  return polyData;
}
