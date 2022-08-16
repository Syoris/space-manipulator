function ax = showFast(obj, parent, collisions, visuals, frames)
    %fastShow HGTransform implementation of rigidBodyTree show
    if isempty(obj.FastVizHelper)
        obj.FastVizHelper= FastVizHelper();
    end

    obj.FastVizHelper.fastUpdate(obj, parent, collisions, visuals, frames);
    
    ax=obj.FastVizHelper.AxesHandle;
end