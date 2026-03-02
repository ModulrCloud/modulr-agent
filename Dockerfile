# Runtime image for modulr_agent
# This takes pre-compiled binaries from CI — it does NOT compile from source.
#
# Multi-arch: binaries are placed in docker-context/linux/{amd64,arm64,arm/v7}/
# by CI before building. docker buildx sets TARGETPLATFORM automatically.

FROM ubuntu:24.04

ARG TARGETPLATFORM

LABEL org.opencontainers.image.source="https://github.com/ModulrCloud/modulr-agent"
LABEL org.opencontainers.image.description="Modulr Robot Agent"
LABEL org.opencontainers.image.licenses="MIT"

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libgstreamer1.0-0 \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        libgstrtspserver-1.0-0 \
        libges-1.0-0 \
        ca-certificates && \
    rm -rf /var/lib/apt/lists/*

RUN groupadd -r modulr && useradd -r -g modulr -G video -d /home/modulr -m modulr

RUN mkdir -p /etc/modulr_agent && \
    chown modulr:modulr /etc/modulr_agent

COPY docker-context/${TARGETPLATFORM}/modulr_agent /usr/bin/modulr_agent
RUN chmod +x /usr/bin/modulr_agent

USER modulr

VOLUME ["/etc/modulr_agent"]

ENTRYPOINT ["/usr/bin/modulr_agent"]
CMD ["start", "--config-override", "/etc/modulr_agent/config.json"]
