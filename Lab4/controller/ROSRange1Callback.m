function ROSRange1Callback(~, message)
    global GL_ranges
    GL_ranges(2) = message.Ranges(1);
end

