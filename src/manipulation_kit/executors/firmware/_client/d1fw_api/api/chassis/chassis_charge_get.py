from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_charge_get_response_200 import ChassisChargeGetResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/chassis/charge",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisChargeGetResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisChargeGetResponse200.from_dict(response.json())

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
) -> Response[ChassisChargeGetResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisChargeGetResponse200 | ErrorEnvelope]:
    """Read the mobile base's charging state

     The same object `GET /v1/chassis/state` carries in its `charge` field, on its own, so an operator
    interface can follow the charging flow at 1 Hz without pulling the whole chassis reading. Four facts
    that routinely disagree travel as four fields rather than as one flag: `dock` is where the base is
    in its docking routine, `auto_charging` is whether automatic charging is the mode it is in,
    `pack`/`charging`/`basis` are what the battery packs say about current, and `dock_point` is which
    waypoint pressing charge would send it to. `charging` is null rather than false whenever nothing
    measured says the base is not charging. Not gated by the chassis soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisChargeGetResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisChargeGetResponse200 | ErrorEnvelope | None:
    """Read the mobile base's charging state

     The same object `GET /v1/chassis/state` carries in its `charge` field, on its own, so an operator
    interface can follow the charging flow at 1 Hz without pulling the whole chassis reading. Four facts
    that routinely disagree travel as four fields rather than as one flag: `dock` is where the base is
    in its docking routine, `auto_charging` is whether automatic charging is the mode it is in,
    `pack`/`charging`/`basis` are what the battery packs say about current, and `dock_point` is which
    waypoint pressing charge would send it to. `charging` is null rather than false whenever nothing
    measured says the base is not charging. Not gated by the chassis soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisChargeGetResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ChassisChargeGetResponse200 | ErrorEnvelope]:
    """Read the mobile base's charging state

     The same object `GET /v1/chassis/state` carries in its `charge` field, on its own, so an operator
    interface can follow the charging flow at 1 Hz without pulling the whole chassis reading. Four facts
    that routinely disagree travel as four fields rather than as one flag: `dock` is where the base is
    in its docking routine, `auto_charging` is whether automatic charging is the mode it is in,
    `pack`/`charging`/`basis` are what the battery packs say about current, and `dock_point` is which
    waypoint pressing charge would send it to. `charging` is null rather than false whenever nothing
    measured says the base is not charging. Not gated by the chassis soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisChargeGetResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ChassisChargeGetResponse200 | ErrorEnvelope | None:
    """Read the mobile base's charging state

     The same object `GET /v1/chassis/state` carries in its `charge` field, on its own, so an operator
    interface can follow the charging flow at 1 Hz without pulling the whole chassis reading. Four facts
    that routinely disagree travel as four fields rather than as one flag: `dock` is where the base is
    in its docking routine, `auto_charging` is whether automatic charging is the mode it is in,
    `pack`/`charging`/`basis` are what the battery packs say about current, and `dock_point` is which
    waypoint pressing charge would send it to. `charging` is null rather than false whenever nothing
    measured says the base is not charging. Not gated by the chassis soft-kill latch.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisChargeGetResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
