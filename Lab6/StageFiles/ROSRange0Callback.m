function ROSRange0Callback(~, message)
    global GL_ranges
    GL_ranges(1) = message.Ranges(1);
end
