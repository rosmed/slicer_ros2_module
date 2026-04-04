#include "vtkAssImpConversion.h"

#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <iostream>

vtkSmartPointer<vtkPolyData> vtkAssImpConversion::vtkAssImpToPolyData(const std::string& filename)
{
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(filename,
                                           aiProcess_Triangulate |
                                           aiProcess_GenNormals |
                                           aiProcess_JoinIdenticalVertices);

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

  // Simple traversal of all meshes in the scene.
  // Warning: This ignores local node transformations. 
  // For standard DAE links in URDFs, usually the mesh root is centered, 
  // but a recursive traversal with applying transforms might be more robust if needed.
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    aiMesh* mesh = scene->mMeshes[i];

    for (unsigned int j = 0; j < mesh->mNumVertices; ++j) {
      aiVector3D pos = mesh->mVertices[j];
      points->InsertNextPoint(pos.x, pos.y, pos.z);

      if (mesh->HasNormals()) {
        aiVector3D n = mesh->mNormals[j];
        normals->InsertNextTuple3(n.x, n.y, n.z);
      }
    }

    for (unsigned int j = 0; j < mesh->mNumFaces; ++j) {
      aiFace face = mesh->mFaces[j];
      if (face.mNumIndices == 3) {
        vtkIdType pts[3] = {
          vertexOffset + face.mIndices[0],
          vertexOffset + face.mIndices[1],
          vertexOffset + face.mIndices[2]
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
