
# Debounce process

1. upon edge, call ISR
1. ISR records value and timestamp of change and restarts 10ms timer.  It does this _every_ time. 
2. Upon timeout, test value again, and then set state (pressed/released).  Emit event.  timer for repeats can also be set.
