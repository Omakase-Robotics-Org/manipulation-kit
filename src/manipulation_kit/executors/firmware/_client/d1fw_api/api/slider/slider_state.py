from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.slider_state_response_200 import SliderStateResponse200
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/slider/state",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | SliderStateResponse200 | None:
    if response.status_code == 200:
        response_200 = SliderStateResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | SliderStateResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | SliderStateResponse200]:
    """Read slider feedback

     A direct read of the slider controller over its serial link (about 100-200 ms). An unreadable slider
    is an error envelope (`502`), unlike the `/v1/state` slot which degrades in place.

    `travel_limits` publishes every layer that constrains travel and the `effective` range they come to
    together, which is the range `set_height` admits: `model` is `[0, travel_max_m]` from the daemon's
    configuration, and `unit` is this robot's bench calibration or `null` when none has been recorded. A
    layer only ever narrows.

    `zero_verification` is separate from `zero_reference` and additive to it. `zero_reference` asks
    whether the drive can vouch for its origin right now; this asks whether the origin this daemon last
    wrote has been read back after a power cycle, which is the only evidence that the drive persisted it
    rather than holding it in RAM. `unrecorded` means this daemon has no record of setting one and is
    not a fault.

    `advisory` is additive and absent unless the origin earns one; a commissioned and verified lift
    never carries one. Nothing on the advisory path writes to the drive.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderStateResponse200]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | SliderStateResponse200 | None:
    """Read slider feedback

     A direct read of the slider controller over its serial link (about 100-200 ms). An unreadable slider
    is an error envelope (`502`), unlike the `/v1/state` slot which degrades in place.

    `travel_limits` publishes every layer that constrains travel and the `effective` range they come to
    together, which is the range `set_height` admits: `model` is `[0, travel_max_m]` from the daemon's
    configuration, and `unit` is this robot's bench calibration or `null` when none has been recorded. A
    layer only ever narrows.

    `zero_verification` is separate from `zero_reference` and additive to it. `zero_reference` asks
    whether the drive can vouch for its origin right now; this asks whether the origin this daemon last
    wrote has been read back after a power cycle, which is the only evidence that the drive persisted it
    rather than holding it in RAM. `unrecorded` means this daemon has no record of setting one and is
    not a fault.

    `advisory` is additive and absent unless the origin earns one; a commissioned and verified lift
    never carries one. Nothing on the advisory path writes to the drive.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderStateResponse200
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | SliderStateResponse200]:
    """Read slider feedback

     A direct read of the slider controller over its serial link (about 100-200 ms). An unreadable slider
    is an error envelope (`502`), unlike the `/v1/state` slot which degrades in place.

    `travel_limits` publishes every layer that constrains travel and the `effective` range they come to
    together, which is the range `set_height` admits: `model` is `[0, travel_max_m]` from the daemon's
    configuration, and `unit` is this robot's bench calibration or `null` when none has been recorded. A
    layer only ever narrows.

    `zero_verification` is separate from `zero_reference` and additive to it. `zero_reference` asks
    whether the drive can vouch for its origin right now; this asks whether the origin this daemon last
    wrote has been read back after a power cycle, which is the only evidence that the drive persisted it
    rather than holding it in RAM. `unrecorded` means this daemon has no record of setting one and is
    not a fault.

    `advisory` is additive and absent unless the origin earns one; a commissioned and verified lift
    never carries one. Nothing on the advisory path writes to the drive.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderStateResponse200]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | SliderStateResponse200 | None:
    """Read slider feedback

     A direct read of the slider controller over its serial link (about 100-200 ms). An unreadable slider
    is an error envelope (`502`), unlike the `/v1/state` slot which degrades in place.

    `travel_limits` publishes every layer that constrains travel and the `effective` range they come to
    together, which is the range `set_height` admits: `model` is `[0, travel_max_m]` from the daemon's
    configuration, and `unit` is this robot's bench calibration or `null` when none has been recorded. A
    layer only ever narrows.

    `zero_verification` is separate from `zero_reference` and additive to it. `zero_reference` asks
    whether the drive can vouch for its origin right now; this asks whether the origin this daemon last
    wrote has been read back after a power cycle, which is the only evidence that the drive persisted it
    rather than holding it in RAM. `unrecorded` means this daemon has no record of setting one and is
    not a fault.

    `advisory` is additive and absent unless the origin earns one; a commissioned and verified lift
    never carries one. Nothing on the advisory path writes to the drive.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderStateResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
