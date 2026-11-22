"""Connect the fixed-step helicopter simulation to the Gallery worker."""

from json import loads

from flight_core import Flight, KEYS


class Runtime:
    def __init__(self):
        self.flight = Flight()
        self.generation = 0

    def step(self, events, elapsed):
        for event, value in events:
            if event == "reset":
                self.flight.reset()
                self.generation = value
            elif event == "press" and value in KEYS:
                self.flight.press(value)
            elif event == "release" and value in KEYS:
                self.flight.release(value)
            elif event == "clear":
                self.flight.clear_input()
        return {"generation": self.generation, "state": loads(self.flight.advance(elapsed))}


def create_runtime(config):
    return Runtime()
