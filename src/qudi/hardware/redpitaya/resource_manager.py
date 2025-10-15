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
import logging
import sys

_pyrpl_lock = Lock()


def patch_pyrpl_logging():
    """
    Monkey-patch pyrpl's LogHandler to fix the emit() method signature issue.
    This fixes the "takes 2 positional arguments but 3 were given" error.
    """
    try:
        # Import pyrpl's widget module to get access to LogHandler
        from pyrpl.widgets import pyrpl_widget

        # Save the original emit method
        original_emit = pyrpl_widget.LogHandler.emit

        # Create a fixed emit method that handles both signatures
        def fixed_emit(self, record, *args, **kwargs):
            """Fixed emit method that accepts extra arguments gracefully."""
            try:
                # Try calling with just the record (correct signature)
                original_emit(self, record)
            except TypeError as e:
                if "takes 2 positional arguments" in str(e):
                    # If there's still a type error, just format and skip the signal
                    try:
                        msg = self.format(record)
                        # Try to emit the signal if possible
                        if hasattr(self, 'log_signal'):
                            self.log_signal.emit([msg])
                    except:
                        # If all else fails, just ignore the log
                        pass
                else:
                    # Re-raise if it's a different error
                    raise

        # Replace the emit method
        pyrpl_widget.LogHandler.emit = fixed_emit
        print("Patched pyrpl LogHandler.emit() method")

    except ImportError:
        # pyrpl not installed or import failed
        pass
    except Exception as e:
        print(f"Failed to patch pyrpl logging: {e}")


def get_pyrpl_instance(hostname, config_name, gui=True):
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
            # Apply the logging patch before creating pyrpl instance
            patch_pyrpl_logging()

            # Create, store, and return new instance
            import pyrpl

            # Temporarily suppress the QtCore.QTimer warning
            import warnings
            with warnings.catch_warnings():
                warnings.filterwarnings("ignore", message=".*QTimer.*")
                warnings.filterwarnings("ignore", message=".*startTimer.*")

                print(f"Creating new pyrpl instance for {hostname} with config {config_name}")

                # Note: gui=True is safe now with the patched logging
                # The QTimer warning will still appear but won't break functionality
                pyrpl_object = pyrpl.Pyrpl(
                    hostname=hostname,
                    config="",  # We don't load a config currently
                    reload_fpga=True,
                    reload_server=True,
                    gui=gui  # Keep GUI enabled
                )

                # If GUI is enabled, minimize the logging verbosity to reduce spam
                if gui and hasattr(pyrpl_object, 'logger'):
                    pyrpl_object.logger.setLevel(logging.WARNING)

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

                # Try to close any GUI windows
                try:
                    if hasattr(pyrpl_object, 'widgets'):
                        for widget in pyrpl_object.widgets:
                            widget.close()
                except:
                    pass

                del _pyrpl_instances[instance_key]
                del pyrpl_object
            else:
                # Update the count
                _pyrpl_instances[instance_key] = (pyrpl_object, owner_count)