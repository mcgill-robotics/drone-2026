class Event:
    def __init__(self):
        self.subscribers = []

    def subscribe(self, callback):
        self.subscribers.append(callback)

    def unsubscribe(self, callback):
        self.subscribers.remove(callback)

    def notify(self, *args, **kwargs):
        for callback in self.subscribers:
            callback(*args, **kwargs)
