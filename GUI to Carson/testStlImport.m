clc
clear variables
close all

%% Initialization
view(gca, [0,5 0]);
axis(gca, [-100 825 -200 200 -70 500]);

link0Patch = makeStlLink('link0.stl', gca, [1, 0.5, 0]);
link0Vertices = get(link0Patch, 'Vertices')';
link0Vertices(4,:) = ones(1, size(link0Vertices,2));

phantomPatch = makeStlLink('phantom.stl', gca, [.9, .75, .6]);
phantomVertices = get(phantomPatch, 'Vertices')';
phantomVertices(4,:) = ones(1, size(phantomVertices,2));
phantomPatch.EdgeColor = 'none';

link1Patch = makeStlLink('link1.stl', gca, [1, 0, 0]);
link1Vertices = get(link1Patch, 'Vertices')';
link1Vertices(4,:) = ones(1, size(link1Vertices,2));

link2Patch = makeStlLink('link2.stl', gca, [0, 0, 1]);
link2Vertices = get(link2Patch, 'Vertices')';
link2Vertices(4,:) = ones(1, size(link2Vertices,2));

link3Patch = makeStlLink('link3.stl', gca, [1, 1, 0]);
link3Vertices = get(link3Patch, 'Vertices')';
link3Vertices(4,:) = ones(1, size(link3Vertices,2));

link4Patch = makeStlLink('link4.stl', gca, [1, 1, 0]);
link4Vertices = get(link4Patch, 'Vertices')';
link4Vertices(4,:) = ones(1, size(link4Vertices,2));

%% updateRobot
[A, A1, A2, A3, A4] = makeHomogeneousTransformations(100,120,-20,10);

% DONE: Use the A matricies to form the T0_n matricies.
T0 = A;
T0_1 = A*A1;
T0_2 = A*A1*A2;
T0_3 = A*A1*A2*A3;
T0_4 = A*A1*A2*A3*A4;

% DONE: Use the T matricies to transform the patch vertices
link0verticesWRTground = T0 * link0Vertices;
phantomverticesWRTground = T0 * phantomVertices;
link1verticesWRTground = T0_1 * link1Vertices;
link2verticesWRTground = T0_2 * link2Vertices;
link3verticesWRTground = T0_3 * link3Vertices;
link4verticesWRTground = T0_4 * link4Vertices;

% DONE: Update the patches with the new vertices
set(link0Patch,'Vertices', link0verticesWRTground(1:3,:)');
set(phantomPatch,'Vertices', phantomverticesWRTground(1:3,:)');
set(link1Patch,'Vertices', link1verticesWRTground(1:3,:)');
set(link2Patch,'Vertices', link2verticesWRTground(1:3,:)');
set(link3Patch,'Vertices', link3verticesWRTground(1:3,:)');
set(link4Patch,'Vertices', link4verticesWRTground(1:3,:)');


