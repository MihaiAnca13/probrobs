function ROSRange2Callback(~, message)
    global GL_ranges
    GL_ranges(3) = message.Ranges(1);
end