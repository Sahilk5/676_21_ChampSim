

/*
    Overall Flow:
    1. Page accessed
    2. anchor it with trigger PC
    3. check if trigger PC has a pattern in SPT
    4. if pattern exists, check if it is good enough to use
    5. if good enough, use the pattern to prefetch
    6. if not good enough, or no pattern, update the pattern with the new
         access and the trigger PC -> Do this when? When the page leaves buffer - for now
*/