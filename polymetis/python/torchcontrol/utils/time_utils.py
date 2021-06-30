import torch


def timestamp_diff(ts1, ts2):
    s_diff = ts1[0] - ts2[0]
    ns_diff = ts1[1] - ts2[1]
    t_diff = s_diff.to(torch.float32) + 1e-9 * ns_diff.to(torch.float32)
    return t_diff


def timestamp_diff_ms(ts1, ts2):
    return 1e3 * timestamp_diff(ts1, ts2)
