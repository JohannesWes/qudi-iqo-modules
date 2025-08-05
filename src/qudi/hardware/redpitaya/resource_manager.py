"""
Shared resources for Red Pitaya hardware modules.
"""

# Global dictionary to store and share pyrpl instances across different Qudi modules
# The key is a unique identifier for the Red Pitaya, e.g., f"{hostname}_{config}"
# The value is the active pyrpl.Pyrpl object.
_pyrpl_instances = {}

# We can also add a lock for thread-safety, although Qudi's module activation
# is typically sequential, it's good practice.
from threading import Lock

_pyrpl_lock = Lock()


def get_pyrpl_instance(hostname, config_name, gui=False):
    """
    Factory function to get or create a shared pyrpl.Pyrpl instance.
    This ensures that only one connection per Red Pitaya is established.
    """
    with _pyrpl_lock:
        instance_key = f"{hostname}_{config_name}"

        if instance_key in _pyrpl_instances:
            # Return existing instance
            pyrpl_object, owner_count = _pyrpl_instances[instance_key]
            owner_count += 1
            _pyrpl_instances[instance_key] = (pyrpl_object, owner_count)
            is_owner = False
            return pyrpl_object, is_owner
        else:
            # Create, store, and return new instance
            import pyrpl
            print(f"Creating new pyrpl instance for {hostname} with config {config_name}")
            pyrpl_object = pyrpl.Pyrpl(
                hostname=hostname,
                config=config_name,
                reload_fpga=True,
                reload_server=True,
                gui=gui
            )
            # Store the instance and a reference count (starting at 1)
            _pyrpl_instances[instance_key] = (pyrpl_object, 1)
            is_owner = True
            return pyrpl_object, is_owner


def release_pyrpl_instance(hostname, config_name):
    """
    Decrements the reference count for a pyrpl instance.
    If the count reaches zero, the instance is cleaned up.
    """
    with _pyrpl_lock:
        instance_key = f"{hostname}_{config_name}"
        if instance_key in _pyrpl_instances:
            pyrpl_object, owner_count = _pyrpl_instances[instance_key]
            owner_count -= 1

            if owner_count <= 0:
                # Last user is gone, so clean up
                print(f"Closing pyrpl instance for {hostname}")
                del _pyrpl_instances[instance_key]
                # It's generally safer to let the Pyrpl object's destructor handle cleanup
                # rather than explicitly calling a close/disconnect method that may not exist.
                del pyrpl_object
            else:
                # Update the count
                _pyrpl_instances[instance_key] = (pyrpl_object, owner_count)