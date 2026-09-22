from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_ultrasonic import ChassisUltrasonic
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/ultrasonic/stream",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisUltrasonic | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisUltrasonic.from_dict(response.text)

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisUltrasonic | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisUltrasonic | ErrorEnvelope]:
    """Watch the mobile base's ultrasonic ring live

     The same reading as `GET /v1/chassis/ultrasonic`, pushed as it changes instead of answered once. The
    response is a `text/event-stream` that stays open until the consumer closes it; each event's `data:`
    is one complete `ChassisUltrasonic` object, sent bare rather than inside the JSON envelope. An event
    named `error` carries a plain message string and the stream continues. Events arrive at up to 10 Hz
    -- a deliberate thinning of the base's own roughly 30 Hz push, sized for a person watching a number
    move -- and an unchanging reading repeats about once a second so that a still stream and a stalled
    one are distinguishable. HOLDING THIS OPEN HAS A COST: the ring reaches the daemon over a vendor
    service that serves ONE client at a time, and for as long as this response is open the daemon holds
    that slot, so the base's other clients -- the vendor's own diagnostics included -- cannot read it.
    Close the stream when nothing is watching. `link` reports which shape is in force and, after the
    held connection ends, when the daemon will try again. Still a HARDWARE-CHECK reading: nothing
    brakes, stops or re-routes the base on it. Not available over WebSocket.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisUltrasonic | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisUltrasonic | ErrorEnvelope | None:
    """Watch the mobile base's ultrasonic ring live

     The same reading as `GET /v1/chassis/ultrasonic`, pushed as it changes instead of answered once. The
    response is a `text/event-stream` that stays open until the consumer closes it; each event's `data:`
    is one complete `ChassisUltrasonic` object, sent bare rather than inside the JSON envelope. An event
    named `error` carries a plain message string and the stream continues. Events arrive at up to 10 Hz
    -- a deliberate thinning of the base's own roughly 30 Hz push, sized for a person watching a number
    move -- and an unchanging reading repeats about once a second so that a still stream and a stalled
    one are distinguishable. HOLDING THIS OPEN HAS A COST: the ring reaches the daemon over a vendor
    service that serves ONE client at a time, and for as long as this response is open the daemon holds
    that slot, so the base's other clients -- the vendor's own diagnostics included -- cannot read it.
    Close the stream when nothing is watching. `link` reports which shape is in force and, after the
    held connection ends, when the daemon will try again. Still a HARDWARE-CHECK reading: nothing
    brakes, stops or re-routes the base on it. Not available over WebSocket.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisUltrasonic | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisUltrasonic | ErrorEnvelope]:
    """Watch the mobile base's ultrasonic ring live

     The same reading as `GET /v1/chassis/ultrasonic`, pushed as it changes instead of answered once. The
    response is a `text/event-stream` that stays open until the consumer closes it; each event's `data:`
    is one complete `ChassisUltrasonic` object, sent bare rather than inside the JSON envelope. An event
    named `error` carries a plain message string and the stream continues. Events arrive at up to 10 Hz
    -- a deliberate thinning of the base's own roughly 30 Hz push, sized for a person watching a number
    move -- and an unchanging reading repeats about once a second so that a still stream and a stalled
    one are distinguishable. HOLDING THIS OPEN HAS A COST: the ring reaches the daemon over a vendor
    service that serves ONE client at a time, and for as long as this response is open the daemon holds
    that slot, so the base's other clients -- the vendor's own diagnostics included -- cannot read it.
    Close the stream when nothing is watching. `link` reports which shape is in force and, after the
    held connection ends, when the daemon will try again. Still a HARDWARE-CHECK reading: nothing
    brakes, stops or re-routes the base on it. Not available over WebSocket.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisUltrasonic | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisUltrasonic | ErrorEnvelope | None:
    """Watch the mobile base's ultrasonic ring live

     The same reading as `GET /v1/chassis/ultrasonic`, pushed as it changes instead of answered once. The
    response is a `text/event-stream` that stays open until the consumer closes it; each event's `data:`
    is one complete `ChassisUltrasonic` object, sent bare rather than inside the JSON envelope. An event
    named `error` carries a plain message string and the stream continues. Events arrive at up to 10 Hz
    -- a deliberate thinning of the base's own roughly 30 Hz push, sized for a person watching a number
    move -- and an unchanging reading repeats about once a second so that a still stream and a stalled
    one are distinguishable. HOLDING THIS OPEN HAS A COST: the ring reaches the daemon over a vendor
    service that serves ONE client at a time, and for as long as this response is open the daemon holds
    that slot, so the base's other clients -- the vendor's own diagnostics included -- cannot read it.
    Close the stream when nothing is watching. `link` reports which shape is in force and, after the
    held connection ends, when the daemon will try again. Still a HARDWARE-CHECK reading: nothing
    brakes, stops or re-routes the base on it. Not available over WebSocket.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisUltrasonic | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
