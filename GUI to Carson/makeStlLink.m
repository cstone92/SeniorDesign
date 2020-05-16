function p = makeStlLink(stlFilename, parentAxes, faceColor)
set(parentAxes, 'DataAspectRatio', [1 1 1]);
stlTR = stlread(stlFilename);
linkVerts = stlTR.Points;
linkFaces = stlTR.ConnectivityList;
p = patch('Parent', parentAxes, 'Faces',linkFaces,'Vertices',linkVerts,'FaceColor',faceColor);
end