"""Desktop-only command interlock. A command or limit switch alone cannot prove load support."""
from dataclasses import dataclass

@dataclass(frozen=True)
class Inputs:
    enabled: bool
    fresh: bool
    air_ok: bool
    stationary: bool
    support_observed: bool
    engaged: bool
    released: bool
    request_move: bool
    limits_ok: bool
    timed_out: bool=False


def update(state, i):
    # No hardware API. The fault output de-energizes the valve and revokes travel.
    # Motor torque can persist only while enable, freshness and independent authority are valid.
    healthy=i.enabled and i.fresh and i.limits_ok
    fault=("fault",healthy and i.support_observed,False,False)
    if state=="fault" or not healthy or not i.air_ok or i.timed_out or (i.engaged and i.released):
        return fault
    if state=="locked":
        if not i.engaged or i.released or not i.stationary:
            return fault
        if i.request_move:
            return "supporting",True,False,False
        return "locked",False,False,False
    if state=="supporting":
        if not i.stationary or not i.engaged or i.released:
            return fault
        if i.request_move and i.support_observed:
            return "releasing",True,True,False
        return ("supporting",True,False,False) if i.request_move else ("locked",False,False,False)
    if state=="releasing":
        if not i.stationary or not i.support_observed:
            return fault
        if not i.request_move:
            return "engaging",True,False,False
        return ("moving",True,True,True) if i.released else ("releasing",True,True,False)
    if state=="moving":
        if not i.released or i.engaged:
            return fault
        if i.request_move:
            return "moving",True,True,True
        return "stopping",True,True,False
    if state=="stopping":
        if not i.released or i.engaged:
            return fault
        if i.stationary and i.support_observed:
            return "engaging",True,False,False
        return "stopping",True,True,False
    if state=="engaging":
        if not i.stationary or not i.support_observed:
            return fault
        return ("locked",False,False,False) if i.engaged else ("engaging",True,False,False)
    raise ValueError("Unknown lift state")
