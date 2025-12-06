"""
Database migration utilities for BlueBox webapp.

Simple migration system that runs SQL scripts on startup to ensure
the database schema is up-to-date.
"""

import logging
from sqlalchemy import text
from database import engine

logger = logging.getLogger(__name__)


# List of migrations to apply in order
# Each migration should be idempotent (safe to run multiple times)
MIGRATIONS = [
    {
        "version": "001",
        "description": "Add flight_name column",
        "sql": """
            ALTER TABLE flight_recordings
            ADD COLUMN IF NOT EXISTS flight_name VARCHAR;
        """
    },
]


def run_migrations():
    """
    Run all database migrations.

    This function is called on application startup to ensure the database
    schema is up-to-date. All migrations use IF NOT EXISTS or similar
    patterns to be idempotent.
    """
    logger.info("Running database migrations...")

    with engine.connect() as conn:
        for migration in MIGRATIONS:
            try:
                logger.info(f"Applying migration {migration['version']}: {migration['description']}")
                conn.execute(text(migration['sql']))
                conn.commit()
                logger.info(f"Migration {migration['version']} applied successfully")
            except Exception as e:
                logger.error(f"Error applying migration {migration['version']}: {e}")
                raise

    logger.info("All migrations completed successfully")


if __name__ == "__main__":
    # Allow running migrations manually
    logging.basicConfig(level=logging.INFO)
    run_migrations()
    print("Migrations completed!")
