classdef BasicGeometry < handle
    %RIGIDBODYGEOMETRY This class defines the shape and pose of a rigid body's 
    %   visual or collision geometry. A RigidBody object can have multiple
    %   RigidBodyGeometry objects.

    properties
        
        %SourceData Information regarding how the geometry is originally
        %   constructed. Specified as a cell array. 
        %   For example,
        %   - {'Box', [0.1 0.2 0.4]}
        %   
        %   Default: {}
        SourceData = {}
        
        %Scale Scaling factor associated with the geometry.
        %   Scale is specified as a row vector of 3 elements defining the
        %   scaling factor along each of the geometry's local frame's axes.
        %
        %   Default: ones(1,3)
        Scale = ones(1,3)

        %Vertices Coordinates of triangle vertices. 
        %   Specified as an NV-by-3 single matrix, where NV is the number
        %   of vertices in the geometry
        %
        %   Default: single([])        
        Vertices = single([])
        
        %Faces List of vertex indices that defines facet triangles.
        %   Specified as an NF-by-3 int32 matrix, where NF is the number of
        %   facets.
        %
        %   Default: int32([])
        Faces = int32([])
        
        %FaceNormals Normals for each facet defined in Faces.
        %   Specified as an NV-by-3 single matrix
        %
        %   Default: single([])
        FaceNormals = single([])
        
        %Color Color of the geometry (RGBA)
        %
        %   Default: [0.5 0.5 0.5 1]
        Color = [0.5 0.5 0.5 1]
        
        %Tform The pose of the geometry frame w.r.t. the rigid body frame
        %
        %   Default: eye(4) 
        Tform = eye(4)
    end
    methods
        function tag = getTag(obj) 
            %getTag
            tag = '';
            src = obj.SourceData;
            if contains(lower(src{1}), {'box', 'cylinder', 'sphere'})
                tag = ['primitive:', src{1}]; 
            end
        end
    end
    
    methods
        
        function set.Vertices(obj, value)
            %set.Vertices
            obj.Vertices = single(value);
        end
        
        function set.Faces(obj, value)
            %set.Faces
            obj.Faces = int32(value);
        end
        
        function set.FaceNormals(obj, value)
            %set.FaceNormals
            obj.FaceNormals = single(value);
        end        
    end
    
end

