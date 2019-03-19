function LeftCallback(~, message)
    global left
    left = message.Ranges(1);
end