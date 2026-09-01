import random
import re
import itertools
import warnings
from functools import cached_property


class CommandGenerator:
    def __init__(self, knowledge):
        self.knowledge = knowledge

        # -----------------------------
        #  COMMAND TEMPLATES
        # -----------------------------
        self.templates = {
            "goToLoc": "{goVerb} {toLocPrep} the {loc_room} then {FOLLOWUP:atLoc}",
            "takeObjFromPlcmt": "{takeVerb} {art} {obj_singCat} {fromLocPrep} the {plcmtLoc} and {FOLLOWUP:hasObj}",
            "findPrsInRoom": "{findVerb} a {gestPers_posePers} {inLocPrep} the {room} and {FOLLOWUP:foundPers}",
            "findObjInRoom": "{findVerb} {art} {obj_singCat} {inLocPrep} the {room} then {FOLLOWUP:foundObj}",
            "meetPrsAtBeac": "{meetVerb} {name} {inLocPrep} the {room} and {FOLLOWUP:foundPers}",
            "countObjOnPlcmt": "{countVerb} {plurCat} there are {onLocPrep} the {plcmtLoc}",
            "countPrsInRoom": "{countVerb} {gestPersPlur_posePersPlur} are {inLocPrep} the {room}",
            "tellPrsInfoInLoc": "{tellVerb} me the {persInfo} of the person {inRoom_atLoc}",
            "tellObjPropOnPlcmt": "{tellVerb} me what is the {objComp} object {onLocPrep} the {plcmtLoc}",
            "talkInfoToGestPrsInRoom": "{talkVerb} {talk} {talkPrep} the {gestPers} {inLocPrep} the {room}",
            "followNameFromBeacToRoom": "{followVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {room}",
            "guideNameFromBeacToBeac": "{guideVerb} {name} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
            "guidePrsFromBeacToBeac": "{guideVerb} the {gestPers_posePers} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
            "guideClothPrsFromBeacToBeac": "{guideVerb} the person wearing {art} {colorClothe} {fromLocPrep} the {loc} {toLocPrep} the {loc_room}",
            "bringMeObjFromPlcmt": "{bringVerb} me {art} {obj} {fromLocPrep} the {plcmtLoc}",
            "tellCatPropOnPlcmt": "{tellVerb} me what is the {objComp} {singCat} {onLocPrep} the {plcmtLoc}",
            "greetClothDscInRm": "{greetVerb} the person wearing {art} {colorClothe} {inLocPrep} the {room} and {FOLLOWUP:foundPers}",
            "greetNameInRm": "{greetVerb} {name} {inLocPrep} the {room} and {FOLLOWUP:foundPers}",
            "meetNameAtLocThenFindInRm": "{meetVerb} {name} {atLocPrep} the {loc} then {findVerb} them {inLocPrep} the {room}",
            "countClothPrsInRoom": "{countVerb} people {inLocPrep} the {room} are wearing {colorClothes}",
            "tellPrsInfoAtLocToPrsAtLoc": "{tellVerb} the {persInfo} of the person {atLocPrep} the {loc} to the person {atLocPrep} the {loc2}",
            "followPrsAtLoc": "{followVerb} the {gestPers_posePers} {inRoom_atLoc}",
        }

        # followup TEMPLATES
        self.followup_templates = {
            "findObj": "{findVerb} {art} {obj_singCat} and {FOLLOWUP:foundObj}",
            "findPrs": "{findVerb} the {gestPers_posePers} and {FOLLOWUP:foundPers}",
            "meetName": "{meetVerb} {name} and {FOLLOWUP:foundPers}",
            "placeObjOnPlcmt": "{placeVerb} it {onLocPrep} the {plcmtLoc2}",
            "putObjInTrash": "throw it in the trash",
            "deliverObjToMe": "{deliverVerb} it to me",
            "deliverObjToPrsInRoom": "{deliverVerb} it {deliverPrep} the {gestPers_posePers} {inLocPrep} the {room}",
            "deliverObjToNameAtBeac": "{deliverVerb} it {deliverPrep} {name} {inLocPrep} the {room}",
            "talkInfo": "{talkVerb} {talk}",
            "followPrs": "{followVerb} them",
            "followPrsToRoom": "{followVerb} them {toLocPrep} the {loc2_room2}",
            "guidePrsToBeacon": "{guideVerb} them {toLocPrep} the {loc2_room2}",
            "takeObj": "{takeVerb} it and {FOLLOWUP:hasObj}",
        }

        # ------------------
        # Possible Followups
        # ------------------
        self.followup_people = {
            "atLoc": ["findPrs", "meetName"],
            "foundPers": ["talkInfo", "followPrs", "followPrsToRoom", "guidePrsToBeacon"],
        }

        self.followup_objects = {
            "atLoc": ["findObj"],
            "foundObj": ["takeObj"],
            "hasObj": ["putObjInTrash","placeObjOnPlcmt", "deliverObjToMe", "deliverObjToPrsInRoom","deliverObjToNameAtBeac"], 
        }

        # -----------------------------
        # COMMAND GROUPS with weigths
        # -----------------------------
        self.person_cmd_list = [
            ("goToLoc", 8),
            ("findPrsInRoom", 4),
            ("meetPrsAtBeac", 4),
            ("countPrsInRoom", 1),
            ("tellPrsInfoInLoc", 1),
            ("talkInfoToGestPrsInRoom", 6),
            ("followNameFromBeacToRoom", 1),
            ("guideNameFromBeacToBeac", 1),
            ("guidePrsFromBeacToBeac", 1),
            ("guideClothPrsFromBeacToBeac", 1),
            ("greetClothDscInRm", 1),
            ("greetNameInRm", 4),
            ("meetNameAtLocThenFindInRm", 1),
            ("countClothPrsInRoom", 1),
            ("tellPrsInfoAtLocToPrsAtLoc", 1),
            ("followPrsAtLoc", 1),
        ]

        self.object_cmd_list = [
            ("goToLoc", 4),
            ("takeObjFromPlcmt", 2),
            ("findObjInRoom", 2),
            ("countObjOnPlcmt", 1),
            ("tellObjPropOnPlcmt", 1),
            ("bringMeObjFromPlcmt", 1),
            ("tellCatPropOnPlcmt", 1),
        ]


        # =====================================================================
    
        self.verb_dict = {
            "takeVerb": ["take", "get", "grasp", "fetch"],
            "placeVerb": ["put", "place"],
            "deliverVerb": ["bring", "give", "deliver"],
            "bringVerb": ["bring", "give"],
            "goVerb": ["go", "navigate"],
            "findVerb": ["find", "locate", "look for"],
            "talkVerb": ["tell", "say"],
            "answerVerb": ["answer"],
            "meetVerb": ["meet"],
            "tellVerb": ["tell"],
            "greetVerb": ["greet", "salute", "say hello to", "introduce yourself to"],
            "rememberVerb": ["meet", "contact", "get to know", "get acquainted with"],
            "countVerb": ["tell me how many"],
            "describeVerb": ["tell me how", "describe"],
            "offerVerb": ["offer"],
            "followVerb": ["follow"],
            "guideVerb": ["guide", "escort", "take", "lead"],
            "accompanyVerb": ["accompany"],
        }

        self.prep_dict = {
            "deliverPrep": ["to"],
            "placePrep": ["on"],
            "inLocPrep": ["in"],
            "fromLocPrep": ["from"],
            "toLocPrep": ["to"],
            "atLocPrep": ["at"],
            "talkPrep": ["to"],
            "locPrep": ["in", "at"],
            "onLocPrep": ["on"],
            "arePrep": ["are"],
            "ofPrsPrep": ["of"],
        }

        self.connector_list = ["and"]

        self.gesture_person_list = [
            "waving person",
            "person raising their left arm",
            "person raising their right arm",
            "person pointing to the left",
            "person pointing to the right",
        ]
        self.pose_person_list = ["sitting person", "standing person", "lying person"]

        self.gesture_person_plural_list = [
            "waving persons",
            "persons raising their left arm",
            "persons raising their right arm",
            "persons pointing to the left",
            "persons pointing to the right",
        ]
        self.pose_person_plural_list = ["sitting persons", "standing persons", "lying persons"]

        self.person_info_list = ["name", "pose", "gesture"]

        self.object_comp_list = [
            "biggest",
            "largest",
            "smallest",
            "heaviest",
            "lightest",
            "thinnest",
        ]

        # Self-referential talk items ("something about yourself", "your teams
        # name/country/affiliation") are deliberately excluded: supporting them
        # was judged unnecessary, so generated corpora skip them (team_info in
        # constants.json holds filler values as a safety net for old corpora).
        self.talk_list = [
            "the time",
            "what day is today",
            "what day is tomorrow",
            "the day of the week",
            "the day of the month",
        ]

        self.color_list = ["blue", "yellow", "black", "white", "red", "orange", "gray"]
        self.clothe_list = ["t shirt", "shirt", "blouse", "sweater", "coat", "jacket"]
        self.clothes_list = ["t shirts", "shirts", "blouses", "sweaters", "coats", "jackets"]

        self.color_clothe_list = [f"{a} {b}" for a, b in itertools.product(self.color_list, self.clothe_list)]
        self.color_clothes_list = [f"{a} {b}" for a, b in itertools.product(self.color_list, self.clothes_list)]


    def generate_command_start(self, cmd_category=""):

        cmd_list = (
            self.person_cmd_list if cmd_category == "people"
            else self.object_cmd_list if cmd_category == "objects"
            else (self.person_cmd_list if random.random() > 0.5 else self.object_cmd_list)
        )

        command = self._weighted_choice(cmd_list)

        if command not in self.templates:
            warnings.warn("Command not covered: " + command)
            return "WARNING"

        template = self.templates[command]

        template = self._resolve_followups(template, cmd_category)
        template = self.insert_all_placeholders(template)
        template = self._resolve_duplicates(template)
        template = self._fix_articles(template)
        
        return template

    def _resolve_followups(self, template, cmd_category):
        matches = re.findall(r"\{FOLLOWUP:(\w+)\}", template)

        for key in matches:
            cmd = self._sample_followup(key, cmd_category)
            expanded = self._generate_followup(cmd, cmd_category)
            template = template.replace(f"{{FOLLOWUP:{key}}}", expanded)

        return template

    def _sample_followup(self, key, cmd_category):
        if cmd_category == "people":
            pool = self.followup_people.get(key, [])
        elif cmd_category == "objects":
            pool = self.followup_objects.get(key, [])
        else:
            pool = self.followup_people.get(key, []) + self.followup_objects.get(key, [])

        choice = random.choice(pool)
        if not choice:
            warnings.warn(f"No followups mapped: '{key}' in, {cmd_category}")

        return choice

    def _generate_followup(self, command, cmd_category=""):

        if command not in self.followup_templates:
            warnings.warn("followup_templates not covered: " + command)
            return "WARNING"

        template = self.followup_templates[command]
        template = self._resolve_followups(template, cmd_category)
        template = self.insert_all_placeholders(template)

        return template

    # ============================================================
    # PLACEHOLDER SYSTEM
    # ============================================================
    def insert_all_placeholders(self, string):
        for ph in re.findall(r"(\{\w+\})", string):
            string = string.replace(ph, self.insert_placeholders(ph))
        return string

    def insert_placeholders(self, ph):
        ph = ph.replace("{", "").replace("}", "")

        if "_" in ph:
            ph = random.choice(ph.split("_"))

        if ph == "name":
            return random.choice(self.knowledge.names)
        elif ph == "room":
            return random.choice(self.knowledge.rooms)
        elif ph == "loc":
            return random.choice(self.knowledge.locations)
        elif ph == "plcmtLoc":
            return random.choice(self.knowledge.placement_locations)
        elif ph == "obj":
            return random.choice(self.knowledge.objects)
        elif ph == "singCat":
            return random.choice(self.knowledge.object_categories_singular)
        elif ph == "plurCat":
            return random.choice(self.knowledge.object_categories_plural)
        
        elif ph == "inRoom":
            return (
                random.choice(self.prep_dict["inLocPrep"])
                + " the "
                + random.choice(self.knowledge.rooms)
            )
        
        elif ph == "atLoc":
            return (
                random.choice(self.prep_dict["atLocPrep"])
                + " the "
                + random.choice(self.knowledge.locations)
            )

        elif ph == "gestPers":
            return random.choice(self.gesture_person_list)
        elif ph == "posePers":
            return random.choice(self.pose_person_list)

        elif ph == "gestPersPlur":
            return random.choice(self.gesture_person_plural_list)
        elif ph == "posePersPlur":
            return random.choice(self.pose_person_plural_list)

        elif ph == "persInfo":
            return random.choice(self.person_info_list)
        elif ph == "objComp":
            return random.choice(self.object_comp_list)

        elif ph == "talk":
            return random.choice(self.talk_list)

        elif ph == "colorClothe":
            return random.choice(self.color_clothe_list)
        elif ph == "colorClothes":
            return random.choice(self.color_clothes_list)

        elif ph == "connector":
            return random.choice(self.connector_list)

        # verbs
        elif ph in self.verb_dict:
            return random.choice(self.verb_dict[ph])

        # prepositions
        elif ph in self.prep_dict:
            return random.choice(self.prep_dict[ph])

        # special placeholders
        elif ph in ["plcmtLoc2", "room2", "loc2"]:
            return "{" + ph +"}"

        elif ph == "art":
            return "{art}"

        warnings.warn("Placeholder not covered: " + ph)
        return "WARNING"

    # ============================================================
    # UTILITIES
    # ============================================================
    def _fix_articles(self, text):
        matches = re.findall(r"\{art\}\s*([A-Za-z])", text)
        if matches:
            return text.replace(
                "{art}",
                "an" if matches[0].lower() in "aeiou" else "a"
            )
        return text

    def _resolve_duplicates(self, text):
        if "{loc2}" in text:
            text = text.replace("{loc2}", random.choice([x for x in self.knowledge.locations if x not in text]))
        if "{room2}" in text:
            text = text.replace("{room2}", random.choice([x for x in self.knowledge.rooms if x not in text]))
        if "{plcmtLoc2}" in text:
            text = text.replace("{plcmtLoc2}", random.choice([x for x in self.knowledge.placement_locations if x not in text]))
        return text

    def _weighted_choice(self, weighted_list):
        items, weights = zip(*weighted_list)
        return random.choices(items, weights=weights, k=1)[0]
