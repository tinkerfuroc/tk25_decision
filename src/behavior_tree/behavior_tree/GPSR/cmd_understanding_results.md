# GPSR Command-Understanding Test

- Generator: official RoboCup@Home `CommandGenerator` (vendored as `_official_cmd_gen.py`)
- Planner: `BtNode_PlanActions` LLM call from `orchestrator.py`
- Model: `openai/gpt-4.1` via OpenRouter
- Commands tested: 30
- Random seed: 42

## Vocabulary fed to generator + planner
- **Rooms:** living_room, office, kitchen, bedroom
- **Locations:** shelf, desk_lamp, bathroom
- **Objects:** kuat, fanta, milk, orange juice, coke, coffee, lime, tangerine, pear, lemon, apple, snack bag, pringles, chocolate, ketchup, oats, mayo, tuna

## Summary

| Metric | Count |
|---|---|
| Fully OK | 25 / 30 |
| Empty plans | 3 |
| LLM/transport errors | 0 |
| Commands with unknown actions | 1 |
| Commands with unknown locations | 0 |
| Commands with unknown objects | 0 |
| Guard-validator rejections | 1 |

## Per-command results

### 1. `tell me how many drinks there are on the shelf`
- **Status:** OK  (3 step(s), 1.77s)
- **Reasoning:** The command asks for the number of drinks on the shelf. The robot should go to the shelf, search for drinks there, count them, and announce the result.
- **Plan:**
  - `goto({"location":"shelf"})`
  - `find_object({"object":"drink","location":"shelf"})`
  - `count({"object":"drink"})`

### 2. `go to the shelf then look for a snack and take it and bring it to me`
- **Status:** OK  (4 step(s), 1.76s)
- **Reasoning:** The command asks the robot to go to the shelf, search for a snack, pick it up, and deliver it to the user. Each clause is mapped to an atomic action: goto for moving to the shelf, find_object for searching for a snack, grasp for picking it up, and deliver to bring it to the user.
- **Plan:**
  - `goto({"location":"shelf"})`
  - `find_object({"object":"snack"})`
  - `grasp({"object":"snack"})`
  - `deliver({"object":"snack","recipient":"you"})`

### 3. `navigate to the bathroom then locate a milk and grasp it and place it on the shelf`
- **Status:** OK  (4 step(s), 1.71s)
- **Reasoning:** The command requires the robot to first navigate to the bathroom, then locate a milk, grasp it, and finally place it on the shelf. Each clause is mapped to an atomic action in the correct order.
- **Plan:**
  - `goto({"location":"bathroom"})`
  - `find_object({"object":"milk"})`
  - `grasp({"object":"milk"})`
  - `place({"location":"shelf"})`

### 4. `introduce yourself to Alex in the kitchen and follow them to the bathroom`
- **Status:** OK  (4 step(s), 1.42s)
- **Reasoning:** The command requires the robot to go to the kitchen, greet Alex (introduce itself), then follow Alex to the bathroom. According to the rules, after following, the robot should go to the specified location.
- **Plan:**
  - `goto({"location":"kitchen"})`
  - `greet({"person":"Alex"})`
  - `follow({"person":"Alex"})`
  - `goto({"location":"bathroom"})`

### 5. `meet Alex in the living_room and follow them`
- **Status:** OK  (3 step(s), 1.56s)
- **Reasoning:** The command asks to meet Alex in the living_room and then follow them. First, the robot should go to the living_room, then find the person named Alex, and finally follow Alex.
- **Plan:**
  - `goto({"location":"living_room"})`
  - `find_person({"descriptor":"Alex"})`
  - `follow({"person":"Alex"})`

### 6. `locate a person pointing to the left in the living_room and tell the day of the month`
- **Status:** OK  (3 step(s), 1.63s)
- **Reasoning:** The command asks to locate a person pointing to the left in the living_room and then tell the day of the month. First, the robot should go to the living_room, then search for the person with the specified descriptor, and finally announce the day of the month.
- **Plan:**
  - `goto({"location":"living_room"})`
  - `find_person({"descriptor":"person pointing to the left"})`
  - `tell_info({"text":"Today is the 6th day of the month."})`

### 7. `go to the desk_lamp then meet Liam and follow them`
- **Status:** OK  (3 step(s), 1.33s)
- **Reasoning:** The command asks to first go to the desk_lamp, then find and meet Liam, and finally follow Liam. Each clause is mapped to an atomic action.
- **Plan:**
  - `goto({"location":"desk_lamp"})`
  - `find_person({"descriptor":"Liam"})`
  - `follow({"person":"Liam"})`

### 8. `navigate to the bathroom then look for the lying person and guide them to the kitchen`
- **Status:** OK  (3 step(s), 1.69s)
- **Reasoning:** The command requires the robot to first navigate to the bathroom, then search for a lying person, and finally guide that person to the kitchen. Each clause is mapped to a corresponding atomic action.
- **Plan:**
  - `goto({"location":"bathroom"})`
  - `find_person({"descriptor":"lying person"})`
  - `guide({"location":"kitchen"})`

### 9. `salute the person wearing a yellow sweater in the office and say the day of the month`
- **Status:** OK  (4 step(s), 2.20s)
- **Reasoning:** The command asks to greet (salute) a specific person (wearing a yellow sweater) in the office and then say the day of the month. First, the robot must go to the office, find the person with the descriptor, greet them, and then announce the day of the month.
- **Plan:**
  - `goto({"location":"office"})`
  - `find_person({"descriptor":"person wearing a yellow sweater"})`
  - `greet({"person":"person wearing a yellow sweater"})`
  - `tell_info({"text":"Today is the Xth day of the month."})`

### 10. `grasp a tuna from the shelf and give it to Sarah in the living_room`
- **Status:** OK  (5 step(s), 1.83s)
- **Reasoning:** The command asks to grasp a tuna from the shelf and deliver it to Sarah in the living_room. The robot must first go to the shelf, find a tuna, grasp it, find Sarah, and deliver the tuna to her in the living_room.
- **Plan:**
  - `goto({"location":"shelf"})`
  - `find_object({"object":"tuna"})`
  - `grasp({"object":"tuna"})`
  - `find_person({"descriptor":"Sarah"})`
  - `deliver({"object":"tuna","recipient":"Sarah","recipient_location":"living_room"})`

### 11. `take a pringles from the shelf and throw it in the trash`
- **Status:** FAIL  (0 step(s), 1.21s)
- **Reasoning:** The command asks to take a pringles from the shelf and throw it in the trash. The robot should go to the shelf, find the pringles, grasp it, but there is no known location 'trash' to place or deliver the object. Therefore, the plan cannot be completed as specified.
- **Plan:**

### 12. `guide Liam from the shelf to the shelf`
- **Status:** FAIL  (0 step(s), 1.37s)
- **Reasoning:** The command asks to guide Liam from the shelf to the shelf, which means the start and end locations are the same. Guiding someone to their current location is redundant and not a meaningful action.
- **Plan:**

### 13. `tell your teams name to the person raising their left arm in the living_room`
- **Status:** OK  (3 step(s), 1.99s)
- **Reasoning:** The command asks to tell the team's name to a specific person (the one raising their left arm) in the living_room. The robot must first go to the living_room, find the person raising their left arm, and then tell them the team's name.
- **Plan:**
  - `goto({"location":"living_room"})`
  - `find_person({"descriptor":"person raising their left arm"})`
  - `tell_info({"text":"Our team name is RoboCup@Home Team."})`

### 14. `salute the person wearing a red jacket in the kitchen and follow them to the bedroom`
- **Status:** OK  (5 step(s), 1.70s)
- **Reasoning:** The command asks to greet (salute) a specific person (wearing a red jacket) in the kitchen, then follow them to the bedroom. First, the robot must go to the kitchen, find the person with the red jacket, greet them, then follow them and go to the bedroom.
- **Plan:**
  - `goto({"location":"kitchen"})`
  - `find_person({"descriptor":"person wearing a red jacket"})`
  - `greet({"person":"person wearing a red jacket"})`
  - `follow({"person":"person wearing a red jacket"})`
  - `goto({"location":"bedroom"})`

### 15. `navigate to the shelf then locate a drink and fetch it and place it on the desk_lamp`
- **Status:** OK  (5 step(s), 1.81s)
- **Reasoning:** The command asks the robot to go to the shelf, find a drink, pick it up, and place it on the desk_lamp. Each clause is mapped to an atomic action: navigate to the shelf, search for a drink, grasp it, go to the desk_lamp, and place it there.
- **Plan:**
  - `goto({"location":"shelf"})`
  - `find_object({"object":"drink"})`
  - `grasp({"object":"drink"})`
  - `goto({"location":"desk_lamp"})`
  - `place({"location":"desk_lamp"})`

### 16. `find a tangerine in the living_room then grasp it and give it to Alex in the kitchen`
- **Status:** OK  (5 step(s), 1.92s)
- **Reasoning:** The command asks to find a tangerine in the living_room, grasp it, and deliver it to Alex in the kitchen. The plan must first navigate to the living_room, search for the tangerine, pick it up, go to the kitchen, and deliver it to Alex.
- **Plan:**
  - `goto({"location":"living_room"})`
  - `find_object({"object":"tangerine"})`
  - `grasp({"object":"tangerine"})`
  - `goto({"location":"kitchen"})`
  - `deliver({"object":"tangerine","recipient":"Alex","recipient_location":"kitchen"})`

### 17. `tell me how many snacks there are on the shelf`
- **Status:** OK  (3 step(s), 1.64s)
- **Reasoning:** The command asks for the number of snacks on the shelf. The robot should go to the shelf, search for the category 'snack', and count the visible instances.
- **Plan:**
  - `goto({"location":"shelf"})`
  - `find_object({"object":"snack","location":"shelf"})`
  - `count({"object":"snack"})`

### 18. `go to the office then locate an apple and grasp it and bring it to the person pointing to the right in the bedroom`
- **Status:** OK  (6 step(s), 1.73s)
- **Reasoning:** The command requires the robot to first go to the office, then find and grasp an apple, then find the person pointing to the right in the bedroom, and finally deliver the apple to that person.
- **Plan:**
  - `goto({"location":"office"})`
  - `find_object({"object":"apple"})`
  - `grasp({"object":"apple"})`
  - `find_person({"descriptor":"person pointing to the right"})`
  - `goto({"location":"bedroom"})`
  - `deliver({"object":"apple","recipient":"person pointing to the right","recipient_location":"bedroom"})`

### 19. `find a chocolate in the bedroom then get it and place it on the desk_lamp`
- **Status:** OK  (4 step(s), 3.11s)
- **Reasoning:** The command asks to find a chocolate in the bedroom, pick it up, and place it on the desk_lamp. This requires searching for the chocolate in the bedroom, grasping it, moving to the desk_lamp, and placing it there.
- **Plan:**
  - `find_object({"object":"chocolate","location":"bedroom"})`
  - `grasp({"object":"chocolate"})`
  - `goto({"location":"desk_lamp"})`
  - `place({"location":"desk_lamp"})`

### 20. `salute Olivia in the bedroom and escort them to the bathroom`
- **Status:** OK  (4 step(s), 1.56s)
- **Reasoning:** The command asks to greet (salute) Olivia in the bedroom and then escort (guide) her to the bathroom. First, the robot must go to the bedroom, find Olivia, greet her, and then guide her to the bathroom.
- **Plan:**
  - `goto({"location":"bedroom"})`
  - `find_person({"descriptor":"Olivia"})`
  - `greet({"person":"Olivia"})`
  - `guide({"location":"bathroom"})`

### 21. `navigate to the kitchen then find the waving person and tell your teams name`
- **Status:** OK  (3 step(s), 1.61s)
- **Reasoning:** The command asks to first navigate to the kitchen, then find a waving person, and finally tell the robot's team name. The plan includes navigating to the kitchen, searching for the waving person, and then announcing the team name.
- **Plan:**
  - `goto({"location":"kitchen"})`
  - `find_person({"descriptor":"waving person"})`
  - `tell_info({"text":"Our team is called RoboCup Home Team."})`

### 22. `go to the office then locate a mayo and take it and throw it in the trash`
- **Status:** OK  (4 step(s), 3.17s)
- **Reasoning:** The command asks to go to the office, find a mayo, pick it up, and throw it in the trash. However, 'trash' is not a known location, so the robot cannot complete the final step. The plan will execute the first steps and then explain why it cannot throw the mayo in the trash.
- **Plan:**
  - `goto({"location":"office"})`
  - `find_object({"object":"mayo"})`
  - `grasp({"object":"mayo"})`
  - `say({"text":"I cannot throw the mayo in the trash because the trash location is unknown."})`

### 23. `meet Olivia in the bedroom and tell your teams country`
- **Status:** FAIL  (3 step(s), 1.50s)
- **Reasoning:** The command asks to meet Olivia in the bedroom and tell the team's country. First, the robot should go to the bedroom. Since 'meet' implies finding Olivia, the robot should search for her in the bedroom. Finally, the robot should state the team's country.
- **Plan:**
  - `goto({"location":"bedroom"})`
  - `find_person({"descriptor":"Olivia"})`
  - `tell_info({"text":"Our team is from <country>."})`
- ⛔ guard rejection: step 2: angle-bracket placeholder '<country>' in params['text']='Our team is from <country>.'. Resolve the literal value or refuse the step — never emit unresolved templates.

### 24. `tell me what is the thinnest object on the shelf`
- **Status:** FAIL  (0 step(s), 1.04s)
- **Reasoning:** The command asks for the thinnest object on the shelf. The robot does not have the capability to determine or compare the thickness of objects, so it cannot execute this request.
- **Plan:**

### 25. `tell the name of the person at the desk_lamp to the person at the bathroom`
- **Status:** FAIL  (1 step(s), 1.50s)
- **Reasoning:** The command requires identifying the person at the desk_lamp, then going to the bathroom and telling the person there the name of the person at the desk_lamp. Since the robot cannot directly identify names, it can say that it cannot execute the request.
- **Plan:**
  - `None({})`
- ⚠ unknown actions: [None]

### 26. `find a lemon in the kitchen then fetch it and deliver it to me`
- **Status:** OK  (4 step(s), 2.00s)
- **Reasoning:** The command asks to find a lemon in the kitchen, fetch it (pick it up), and deliver it to the user. The plan must include navigating to the kitchen, searching for a lemon, grasping it, and delivering it to the user.
- **Plan:**
  - `goto({"location":"kitchen"})`
  - `find_object({"object":"lemon","location":"kitchen"})`
  - `grasp({"object":"lemon"})`
  - `deliver({"object":"lemon","recipient":"me"})`

### 27. `tell me what is the largest drink on the desk_lamp`
- **Status:** OK  (2 step(s), 1.62s)
- **Reasoning:** The command asks for the largest drink on the desk_lamp. The robot should search for drinks at the desk_lamp, but it cannot directly determine which is the largest. Since the robot cannot resolve 'largest' without additional perception or clarification, it should explain this limitation.
- **Plan:**
  - `find_object({"object":"drink","location":"desk_lamp"})`
  - `say({"text":"I can find drinks on the desk_lamp, but I cannot determine which one is the largest."})`

### 28. `navigate to the bathroom then locate a fruit and get it and bring it to Sarah in the living_room`
- **Status:** OK  (5 step(s), 3.54s)
- **Reasoning:** The command asks the robot to first navigate to the bathroom, then find a fruit, grasp it, and deliver it to Sarah in the living_room. Each clause is mapped to an atomic action, following the required order.
- **Plan:**
  - `goto({"location":"bathroom"})`
  - `find_object({"object":"fruit"})`
  - `grasp({"object":"fruit"})`
  - `goto({"location":"living_room"})`
  - `deliver({"object":"fruit","recipient":"Sarah","recipient_location":"living_room"})`

### 29. `say hello to the person wearing a black jacket in the office and follow them`
- **Status:** OK  (4 step(s), 1.61s)
- **Reasoning:** The command asks to greet a specific person (wearing a black jacket) in the office and then follow them. First, the robot should go to the office, then find the person matching the descriptor, greet them, and finally follow them.
- **Plan:**
  - `goto({"location":"office"})`
  - `find_person({"descriptor":"person wearing a black jacket"})`
  - `greet({"person":"person wearing a black jacket"})`
  - `follow({"person":"person wearing a black jacket"})`

### 30. `meet Sarah in the kitchen and follow them`
- **Status:** OK  (3 step(s), 2.17s)
- **Reasoning:** The command asks to meet Sarah in the kitchen and then follow her. First, the robot should go to the kitchen. Then, it should find the person named Sarah. After locating Sarah, the robot should follow her.
- **Plan:**
  - `goto({"location":"kitchen"})`
  - `find_person({"descriptor":"Sarah"})`
  - `follow({"person":"Sarah"})`
