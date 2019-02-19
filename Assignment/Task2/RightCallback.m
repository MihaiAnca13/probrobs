function RightCallback(~, message)
    global right
    right = message.Ranges(1);
end