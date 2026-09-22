from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.slider_set_limits_request import SliderSetLimitsRequest
from ...models.slider_set_limits_response_200 import SliderSetLimitsResponse200
from ...types import Response


def _get_kwargs(
    *,
    body: SliderSetLimitsRequest,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/slider/set_limits",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | SliderSetLimitsResponse200 | None:
    if response.status_code == 200:
        response_200 = SliderSetLimitsResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | SliderSetLimitsResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: SliderSetLimitsRequest,
) -> Response[ErrorEnvelope | SliderSetLimitsResponse200]:
    """COMMISSIONING ONLY: record how far THIS robot's lift may travel

     **Not a motion verb and not a configuration edit.** Nothing is written to the drive and nothing
    moves; what is written is the daemon's own state file, and what it decides is the range every later
    `set_height` is admitted against. It is the unit layer of `slider/state.travel_limits`: what one
    machine's jig, cabling and mechanical build turned out to allow, measured on a bench once per robot.

    A layer may only NARROW. `lower_m` below `0`, or `upper_m` above the model ceiling `[slider]
    travel_max_m`, is a `400` — a calibration that claimed more travel than the model has would be a
    calibration that widened it.

    Two gates, both required, and the same two `slider/set_zero` has. The daemon must have been started
    with `[slider] commissioning = true` (otherwise `409`, before the body is parsed), and the body must
    carry `{"confirm": "SET_LIMITS"}` (otherwise `400`). The slider's soft-kill latch applies.

    The record survives a restart: it is in `/var/lib/d1-firmwared/slider.toml`, or wherever `[slider]
    state_file` points. A daemon that keeps no state file answers `503` rather than accepting a fact it
    would forget. The response is the state read back, so the new effective range needs no second call.

    Args:
        body (SliderSetLimitsRequest): `POST /v1/slider/set_limits`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderSetLimitsResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: SliderSetLimitsRequest,
) -> ErrorEnvelope | SliderSetLimitsResponse200 | None:
    """COMMISSIONING ONLY: record how far THIS robot's lift may travel

     **Not a motion verb and not a configuration edit.** Nothing is written to the drive and nothing
    moves; what is written is the daemon's own state file, and what it decides is the range every later
    `set_height` is admitted against. It is the unit layer of `slider/state.travel_limits`: what one
    machine's jig, cabling and mechanical build turned out to allow, measured on a bench once per robot.

    A layer may only NARROW. `lower_m` below `0`, or `upper_m` above the model ceiling `[slider]
    travel_max_m`, is a `400` — a calibration that claimed more travel than the model has would be a
    calibration that widened it.

    Two gates, both required, and the same two `slider/set_zero` has. The daemon must have been started
    with `[slider] commissioning = true` (otherwise `409`, before the body is parsed), and the body must
    carry `{"confirm": "SET_LIMITS"}` (otherwise `400`). The slider's soft-kill latch applies.

    The record survives a restart: it is in `/var/lib/d1-firmwared/slider.toml`, or wherever `[slider]
    state_file` points. A daemon that keeps no state file answers `503` rather than accepting a fact it
    would forget. The response is the state read back, so the new effective range needs no second call.

    Args:
        body (SliderSetLimitsRequest): `POST /v1/slider/set_limits`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderSetLimitsResponse200
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: SliderSetLimitsRequest,
) -> Response[ErrorEnvelope | SliderSetLimitsResponse200]:
    """COMMISSIONING ONLY: record how far THIS robot's lift may travel

     **Not a motion verb and not a configuration edit.** Nothing is written to the drive and nothing
    moves; what is written is the daemon's own state file, and what it decides is the range every later
    `set_height` is admitted against. It is the unit layer of `slider/state.travel_limits`: what one
    machine's jig, cabling and mechanical build turned out to allow, measured on a bench once per robot.

    A layer may only NARROW. `lower_m` below `0`, or `upper_m` above the model ceiling `[slider]
    travel_max_m`, is a `400` — a calibration that claimed more travel than the model has would be a
    calibration that widened it.

    Two gates, both required, and the same two `slider/set_zero` has. The daemon must have been started
    with `[slider] commissioning = true` (otherwise `409`, before the body is parsed), and the body must
    carry `{"confirm": "SET_LIMITS"}` (otherwise `400`). The slider's soft-kill latch applies.

    The record survives a restart: it is in `/var/lib/d1-firmwared/slider.toml`, or wherever `[slider]
    state_file` points. A daemon that keeps no state file answers `503` rather than accepting a fact it
    would forget. The response is the state read back, so the new effective range needs no second call.

    Args:
        body (SliderSetLimitsRequest): `POST /v1/slider/set_limits`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderSetLimitsResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: SliderSetLimitsRequest,
) -> ErrorEnvelope | SliderSetLimitsResponse200 | None:
    """COMMISSIONING ONLY: record how far THIS robot's lift may travel

     **Not a motion verb and not a configuration edit.** Nothing is written to the drive and nothing
    moves; what is written is the daemon's own state file, and what it decides is the range every later
    `set_height` is admitted against. It is the unit layer of `slider/state.travel_limits`: what one
    machine's jig, cabling and mechanical build turned out to allow, measured on a bench once per robot.

    A layer may only NARROW. `lower_m` below `0`, or `upper_m` above the model ceiling `[slider]
    travel_max_m`, is a `400` — a calibration that claimed more travel than the model has would be a
    calibration that widened it.

    Two gates, both required, and the same two `slider/set_zero` has. The daemon must have been started
    with `[slider] commissioning = true` (otherwise `409`, before the body is parsed), and the body must
    carry `{"confirm": "SET_LIMITS"}` (otherwise `400`). The slider's soft-kill latch applies.

    The record survives a restart: it is in `/var/lib/d1-firmwared/slider.toml`, or wherever `[slider]
    state_file` points. A daemon that keeps no state file answers `503` rather than accepting a fact it
    would forget. The response is the state read back, so the new effective range needs no second call.

    Args:
        body (SliderSetLimitsRequest): `POST /v1/slider/set_limits`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderSetLimitsResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
