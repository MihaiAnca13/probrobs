function FwdCallback(~, message)
    global fwd
    fwd = message.Ranges(1);
end