FROM python:3.11-slim

WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends ffmpeg libglib2.0-0 libgl1 \
    && rm -rf /var/lib/apt/lists/*

COPY backend/requirements.txt backend/requirements-usd.txt /app/
ARG INSTALL_USD=true
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --disable-pip-version-check --prefer-binary --timeout 90 --retries 8 -r /app/requirements.txt \
    && if [ "$INSTALL_USD" = "true" ]; then \
         pip install --disable-pip-version-check --prefer-binary --timeout 90 --retries 8 -r /app/requirements-usd.txt; \
       fi \
    && pip check

COPY backend /app/backend
COPY scripts/realsense_d435i_bridge.py /app/scripts/realsense_d435i_bridge.py

EXPOSE 9000

CMD ["uvicorn", "backend.server:app", "--host", "0.0.0.0", "--port", "9000", "--reload"]
