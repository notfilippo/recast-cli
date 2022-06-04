FROM alpine:latest AS builder

RUN apk --no-cache add git cmake clang clang-dev make gcc g++ libc-dev linux-headers

WORKDIR /base
COPY . /base

WORKDIR /base/build
RUN cmake ..

RUN make

RUN ls

FROM alpine:latest

RUN apk --no-cache add libstdc++ libgcc

COPY --from=builder /base/build/RecastCli ./

ENTRYPOINT ["./RecastCli"]